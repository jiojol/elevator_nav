#include <plan_manage/scan_replan_fsm.h>
#include <plan_manage/terminal_reference.hpp>
#include <plan_manage/start_alignment.hpp>
#include <plan_manage/local_target.hpp>
#include <cmath>
#include <boost/make_shared.hpp>
#include <sstream>
#include <stdexcept>

namespace scan_planner {
void SCANReplanFSM::init(ros::NodeHandle& nh) {
  nh.param("fsm/navi_mode",navi_mode_,3);
  nh.param("fsm/planning_horizon",horizon_,3.5);
  nh.param("fsm/thresh_replan",replan_distance_,1.);
  nh.param("fsm/thresh_no_replan",no_replan_distance_,.1);
  nh.param("fsm/emergency_time_",emergency_time_,1.);
  nh.param("fsm/fail_safe",fail_safe_,true);
  nh.param("fsm/odom_twist_in_body_frame",twist_in_body_,false);
  nh.param("grid_map/body_height",body_height_,.4);
  double turn_max, turn_accel;
  ros::param::param("/scan_command_gate/start_turn_max_speed",turn_max,.5);
  ros::param::param("/scan_command_gate/start_turn_accel",turn_accel,1.6);
  if (!std::isfinite(turn_max)||turn_max<=0.||turn_max>1.||!std::isfinite(turn_accel)||turn_accel<=0.)
    throw std::invalid_argument("invalid start alignment speed/acceleration");
  // Cover a permit lifetime plus the commanded acceleration stopping angle.
  // This is a geometric allowance, not a measured physical braking guarantee.
  alignment_padding_=.5*turn_max*turn_max/turn_accel+turn_max*.2+.02;
  ros::param::param("/dog_nav/geometry/reference_offset_x",reference_offset_.x(),0.);
  ros::param::param("/dog_nav/geometry/reference_offset_y",reference_offset_.y(),0.);
  if (!reference_offset_.allFinite()) throw std::invalid_argument("invalid navigation reference offset");
  nh.param<std::string>("grid_map/frame_id",frame_,"map");
  if (!std::isfinite(horizon_)||horizon_<=.2||!std::isfinite(emergency_time_)||emergency_time_<=0.
      || !std::isfinite(replan_distance_)||replan_distance_<=0.||!std::isfinite(body_height_)) throw std::invalid_argument("invalid local planner configuration");
  if (navi_mode_!=1 && navi_mode_!=3) throw std::invalid_argument("local planner accepts manual targets (1) or global paths (3)");
  visualization_.reset(new PlanningVisualization(nh));
  manager_.reset(new SCANPlannerManager);manager_->initPlanModules(nh,visualization_);
  std::string pose_topic;ros::param::param<std::string>("/body_pose_topic",pose_topic,"/dog_nav/odometry");
  odom_sub_=nh.subscribe(pose_topic,1,&SCANReplanFSM::odometry,this);
  if (navi_mode_==3) route_sub_=nh.subscribe("/initial_path",1,&SCANReplanFSM::path,this);
  else goal_sub_=nh.subscribe("/move_base_simple/goal",1,&SCANReplanFSM::goal,this);
  alignment_pub_=nh.advertise<std_msgs::String>("/planning/start_alignment",1);
  alignment_done_sub_=nh.subscribe("/planning/start_alignment_done",1,&SCANReplanFSM::alignmentDone,this);
  frozen_sub_=nh.subscribe("/planning/go2_execution_frozen",10,&SCANReplanFSM::frozen,this);
  handoff_sub_=nh.subscribe("/planning/local_segment_end",10,&SCANReplanFSM::handoff,this);
  hold_sub_=nh.subscribe("/planning/collision_control",20,&SCANReplanFSM::hold,this);
  position_reached_sub_=nh.subscribe("/planning/goal_position_reached",10,&SCANReplanFSM::arrived,this);
  cancel_sub_=nh.subscribe("/plan_map/cancel_scan",1,&SCANReplanFSM::cancel,this);
  spline_pub_=nh.advertise<scan_planner::Bspline>("/planning/bspline",10);
  tracking_path_pub_=nh.advertise<nav_msgs::Path>("/planning/tracking_path",1,true);
  collision_pub_=nh.advertise<std_msgs::String>("/planning/collision_stop",10);
  status_pub_=nh.advertise<std_msgs::String>("/planning/local_planner_status",1,true);
  freeze_updated_=ros::Time::now();cancel_stamp_=freeze_updated_;
  exec_timer_=nh.createTimer(ros::Duration(.01),&SCANReplanFSM::execute,this);
  collision_timer_=nh.createTimer(ros::Duration(.05),&SCANReplanFSM::collision,this);
  status("ready: prior-map constrained local planner");
}
void SCANReplanFSM::status(const std::string& reason) {
  static const char* names[]={"WAIT_TARGET","GENERATE","EXECUTE","REPLAN","EMERGENCY","ALIGN_START"};
  std::ostringstream out;out.precision(16);out<<"{\"state\":\""<<names[state_]<<"\",\"reason\":\""<<reason
      <<"\",\"stamp\":"<<ros::Time::now().toSec()<<",\"trajectory_id\":"<<manager_->local_data_.traj_id_
      <<",\"prior_required\":"<<(manager_->grid_map_->require_plan_free_?"true":"false")
      <<",\"last_solve_ms\":"<<last_solve_ms_<<",\"collision_distance\":"<<collision_distance_
      <<",\"braking_distance\":"<<braking_distance_<<"}";
  std_msgs::String msg;msg.data=out.str();status_pub_.publish(msg);
}
void SCANReplanFSM::transition(State next,const char* reason) {
  if (state_==next) return;
  state_=next;status(reason);ROS_INFO("SCAN state=%d: %s",int(state_),reason);
}
bool SCANReplanFSM::freshOdom() const {
  const auto now=ros::Time::now();const double age=(now-odom_stamp_).toSec();
  return have_odom_ && age>=-.1 && age<=.5 && (now-odom_received_).toSec()<=.5;
}
double SCANReplanFSM::yaw() const {
  const auto r=orientation_.toRotationMatrix();return std::atan2(r(1,0),r(0,0));
}
void SCANReplanFSM::odometry(const nav_msgs::OdometryConstPtr& msg) {
  if (msg->header.frame_id!=frame_) { have_odom_=false;return; }
  const auto& p=msg->pose.pose.position;const auto& q=msg->pose.pose.orientation;
  Eigen::Vector3d position(p.x,p.y,p.z),velocity(msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z);
  Eigen::Quaterniond orientation(q.w,q.x,q.y,q.z);
  if (!position.allFinite()||!velocity.allFinite()||!orientation.coeffs().allFinite()||orientation.norm()<1e-9) { have_odom_=false;return; }
  position_=position;orientation_=orientation.normalized();velocity_=twist_in_body_?orientation_*velocity:velocity;
  odom_stamp_=msg->header.stamp;odom_received_=ros::Time::now();have_odom_=true;
}
void SCANReplanFSM::path(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses.empty()) { cancel(std_msgs::EmptyConstPtr());return; }
  if (msg->header.frame_id!=frame_ || msg->header.stamp<=cancel_stamp_ || msg->header.stamp<=route_stamp_) return;
  std::vector<Eigen::Vector3d> points;
  for (const auto& pose:msg->poses) {
    if (!pose.header.frame_id.empty() && pose.header.frame_id!=frame_) return;
    const auto& p=pose.pose.position;points.emplace_back(p.x,p.y,p.z+body_height_);
  }
  const auto& q=msg->poses.back().pose.orientation;
  Eigen::Quaterniond goal_orientation(q.w,q.x,q.y,q.z);
  if (!goal_orientation.coeffs().allFinite()||goal_orientation.norm()<1e-9) return;
  const Eigen::Matrix3d goal_rotation=goal_orientation.normalized().toRotationMatrix();
  const double goal_yaw=std::atan2(goal_rotation(1,0),goal_rotation(0,0));
  const auto adapted=terminalReference(points,goal_yaw,reference_offset_);
  ReferencePath candidate;if (!candidate.set(adapted)) { ROS_ERROR("Invalid global path; existing goal unchanged.");return; }
  // Same goal stamp, separate tracking reference. The execution adapter keeps
  // the original requested pose and checks the corresponding body center.
  nav_msgs::Path tracking;tracking.header=msg->header;
  for (const auto& p:adapted) {
    geometry_msgs::PoseStamped pose;pose.header=tracking.header;
    pose.pose.position.x=p.x();pose.pose.position.y=p.y();pose.pose.position.z=p.z()-body_height_;
    pose.pose.orientation.w=1.;tracking.poses.push_back(pose);
  }
  tracking.poses.back().pose.orientation=q;
  tracking_path_pub_.publish(tracking);
  if(alignment_active_)publishStartAlignment(false,true);alignment_active_=false;
  route_=candidate;route_stamp_=msg->header.stamp;progress_=0.;projected_=false;
  failures_=0;have_target_=true;manager_->local_reference_.clear();
  if (stop_latched_) { transition(EMERGENCY,"new route waiting for stop acknowledgement");return; }
  waiting_=false;transition(GENERATE,"new global route");
}
void SCANReplanFSM::goal(const geometry_msgs::PoseStampedConstPtr& msg) {
  if (!freshOdom()||msg->header.frame_id!=frame_) return;
  nav_msgs::Path path;path.header=msg->header;path.header.stamp=ros::Time::now();
  geometry_msgs::PoseStamped a=*msg,b=*msg;
  a.pose.position.x=position_.x();a.pose.position.y=position_.y();a.pose.position.z=position_.z()-body_height_;
  b.pose.position.z=position_.z()-body_height_;path.poses={a,b};
  this->path(boost::make_shared<nav_msgs::Path>(path));
}
void SCANReplanFSM::cancel(const std_msgs::EmptyConstPtr&) {
  if(alignment_active_)publishStartAlignment(false,true);alignment_active_=false;
  cancel_stamp_=ros::Time::now();have_target_=false;route_.clear();manager_->local_reference_.clear();
  nav_msgs::Path empty;empty.header.stamp=cancel_stamp_;empty.header.frame_id=frame_;tracking_path_pub_.publish(empty);
  if (have_odom_) { manager_->EmergencyStop(position_);publishTrajectory(false); }
  // An in-flight physical stop must still be acknowledged after a cancel.
  transition(stop_latched_?EMERGENCY:WAIT_TARGET,"goal cancelled");
}
void SCANReplanFSM::arrived(const std_msgs::HeaderConstPtr& msg) {
  if (!have_target_ || msg->stamp!=route_stamp_) return;
  have_target_=false;manager_->local_reference_.clear();transition(WAIT_TARGET,"position reached; hardware adapter owns final yaw");
}
void SCANReplanFSM::updateClock() {
  const auto now=ros::Time::now();const double dt=(now-freeze_updated_).toSec();freeze_updated_=now;
  if (frozen_ && dt>0. && dt<=.2 && manager_->local_data_.start_time_.toSec()>0.) manager_->local_data_.start_time_+=ros::Duration(dt);
}
void SCANReplanFSM::frozen(const std_msgs::BoolConstPtr& msg) { updateClock();frozen_=msg->data; }
void SCANReplanFSM::handoff(const std_msgs::Int64ConstPtr& msg) {
  if (have_target_ && state_==EXECUTE && msg->data==manager_->local_data_.traj_id_) transition(REPLAN,"measured segment handoff");
}
void SCANReplanFSM::hold(const std_msgs::StringConstPtr& msg) {
  if (msg->data=="hold") stop("hardware execution hold",false);
  else if (msg->data=="replan" && stop_latched_) {
    stop_latched_=false;waiting_=have_target_;failures_=0;retry_at_=ros::Time::now();
    transition(have_target_?GENERATE:WAIT_TARGET,"hardware stop acknowledged");
  }
}
bool SCANReplanFSM::selectTarget() {
  if (route_.empty()||!manager_->grid_map_->hasPlanFree()) return false;
  const double traveled=projected_?(position_-last_projection_pose_).norm():.5;
  const auto projection=route_.project(position_,std::max(0.,progress_-.35),std::min(route_.length(),progress_+traveled+.5),manager_->grid_map_->terrainEnabled());
  progress_=std::max(progress_,projection.arc);last_projection_pose_=position_;projected_=true;
  const double target=localTargetArc(route_,start_,progress_,horizon_,manager_->pp_.ctrl_pt_dist,
      manager_->grid_map_->getResolution(),[&](const Eigen::Vector3d& p,double h) {
        return manager_->grid_map_->getInflateOccupancy(p,h)==0;
      },manager_->grid_map_->terrainEnabled());
  if (target<0.) return false;
  target_=route_.at(target);target_velocity_=route_.tangent(target)*manager_->pp_.max_vel_;
  if (route_.length()-target<manager_->pp_.max_vel_*manager_->pp_.max_vel_/(2.*manager_->pp_.max_acc_)) target_velocity_.setZero();
  const double seed_arc=route_.project(start_,std::max(0.,progress_-.35),std::min(route_.length(),progress_+.5),manager_->grid_map_->terrainEnabled()).arc;
  manager_->local_reference_=route_.vertices(seed_arc,target);
  if (manager_->local_reference_.empty()) return false;
  manager_->local_reference_.front()=start_;
  return true;
}
bool SCANReplanFSM::plan(bool from_trajectory,bool random) {
  if (!freshOdom()||!manager_->grid_map_->hasFusedCloud()) return false;
  const auto solve_started=ros::SteadyTime::now();
  start_=position_;start_velocity_=velocity_;start_acceleration_.setZero();
  if (from_trajectory && !waiting_ && manager_->local_data_.duration_>0.) {
    auto& info=manager_->local_data_;const double t=std::max(0.,std::min(info.duration_,(ros::Time::now()-info.start_time_).toSec()));
    const auto scheduled=info.position_traj_.evaluateDeBoorT(t);
    if ((scheduled-position_).norm()<=.2) {
      // Preserve useful acceleration continuity, but the new curve must
      // begin at the measured body position and velocity, not ahead of it.
      start_acceleration_=info.acceleration_traj_.evaluateDeBoorT(t);
    }
  }
  if (!selectTarget()) {
    last_solve_ms_=1000.*(ros::SteadyTime::now()-solve_started).toSec();
    return false;
  }
  manager_->measured_start_yaw_=yaw();
  const bool success=manager_->reboundReplan(start_,start_velocity_,start_acceleration_,target_,target_velocity_,!from_trajectory,random);
  last_solve_ms_=1000.*(ros::SteadyTime::now()-solve_started).toSec();
  if (!success) return false;
  failures_=0;waiting_=false;publishTrajectory();
  status("prior-constrained initialization accepted");return true;
}
bool SCANReplanFSM::replan() {
  // Native SCAN sequence: reuse seed, polynomial seed, randomized seed.
  // The route initializer is used for the deterministic fresh seed; random
  // initialization still uses native polynomial perturbation.
  return plan(true,false)||plan(false,false)||plan(false,true);
}
bool SCANReplanFSM::beginStartAlignment() {
  if(alignment_active_||!freshOdom()||velocity_.head<2>().norm()>.03||route_.empty()||!manager_->grid_map_->hasPlanFree()||!manager_->grid_map_->hasFusedCloud())return false;
  const double turn_minimum=manager_->grid_map_->terrainEnabled()?.5*manager_->grid_map_->getResolution():.4;
  if((route_.at(route_.length())-position_).head<2>().norm()<turn_minimum)return false;
  const double traveled=projected_?(position_-last_projection_pose_).norm():.5;
  const double arc=route_.project(position_,std::max(0.,progress_-.35),std::min(route_.length(),progress_+traveled+.5),manager_->grid_map_->terrainEnabled()).arc;
  const auto ahead=route_.at(std::min(route_.length(),arc+.65));
  const auto plan=chooseStartAlignment(position_,yaw(),ahead,[&](const Eigen::Vector3d& p,double h){return manager_->grid_map_->getInflateOccupancy(p,h)==0;},2.,alignment_padding_,manager_->grid_map_->terrainEnabled()?turn_minimum:.2);
  if(!plan.required)return false;
  if(!plan.allowed){ROS_WARN_THROTTLE(2.,"Reverse route needs a turn but the native prior/obstacle sweep is blocked; trying normal moving initialization.");return false;}
  alignment_active_=true;++alignment_sequence_;alignment_anchor_=position_;alignment_start_yaw_=yaw();alignment_target_=plan.target_yaw;alignment_angle_=plan.signed_angle;
  transition(ALIGN_START,"reverse route: checked stationary start alignment");publishStartAlignment(true);return true;
}
void SCANReplanFSM::publishStartAlignment(bool allowed,bool cancel) {
  std::ostringstream out;out.precision(17);
  out<<"{\"goal_stamp_ns\":"<<route_stamp_.toNSec()<<",\"request_id\":\""<<route_stamp_.toNSec()<<":"<<alignment_sequence_
     <<"\",\"frame\":\""<<frame_<<"\",\"stamp\":"<<ros::Time::now().toSec()<<",\"allowed\":"<<(allowed?"true":"false")
     <<",\"cancel\":"<<(cancel?"true":"false")<<",\"start_yaw\":"<<alignment_start_yaw_<<",\"target_yaw\":"<<alignment_target_
     <<",\"current_yaw\":"<<yaw()<<",\"remaining_angle\":"<<alignmentRemaining(alignment_target_,yaw(),alignment_angle_)<<",\"signed_angle\":"<<alignment_angle_<<",\"x\":"<<position_.x()<<",\"y\":"<<position_.y()<<",\"z\":"<<position_.z()<<"}";
  std_msgs::String message;message.data=out.str();alignment_pub_.publish(message);alignment_last_publish_=ros::Time::now();
}
void SCANReplanFSM::updateStartAlignment() {
  if(!alignment_active_)return;
  if((position_-alignment_anchor_).norm()>.12){publishStartAlignment(false,true);alignment_active_=false;transition(GENERATE,"start alignment position changed; regenerate from feedback");return;}
  if((ros::Time::now()-alignment_last_publish_).toSec()<.05)return;
  const double remaining=alignmentRemaining(alignment_target_,yaw(),alignment_angle_);
  const bool free=manager_->grid_map_->hasFusedCloud()&&alignmentArcFree(position_,yaw(),remaining,alignment_padding_,[&](const Eigen::Vector3d& p,double h){return manager_->grid_map_->getInflateOccupancy(p,h)==0;});
  publishStartAlignment(free);
}
void SCANReplanFSM::alignmentDone(const std_msgs::StringConstPtr& msg) {
  if(state_!=ALIGN_START||!alignment_active_||msg->data!=std::to_string(route_stamp_.toNSec())+":"+std::to_string(alignment_sequence_)||!freshOdom())return;
  if(std::abs(alignmentWrap(alignment_target_-yaw()))>.3)return;
  alignment_active_=false;waiting_=true;failures_=0;transition(GENERATE,"start heading aligned; generate walking trajectory from measured state");
}
void SCANReplanFSM::publishTrajectory(bool obstacle_hold) {
  collision_progress_.reset(manager_->local_data_.position_traj_,manager_->grid_map_->getResolution(),manager_->grid_map_->terrainEnabled());
  auto& info=manager_->local_data_;scan_planner::Bspline msg;
  msg.order=3;msg.start_time=info.start_time_;msg.traj_id=info.traj_id_;msg.obstacle_hold=obstacle_hold;
  const auto controls=info.position_traj_.getControlPoint();const auto knots=info.position_traj_.getKnot();
  for (int i=0;i<controls.cols();++i) { geometry_msgs::Point p;p.x=controls(0,i);p.y=controls(1,i);p.z=controls(2,i);msg.pos_pts.push_back(p); }
  for (int i=0;i<knots.size();++i) msg.knots.push_back(knots[i]);
  spline_pub_.publish(msg);
  if (!obstacle_hold) visualization_->displayOptimalTraj(info.position_traj_,0);
}
void SCANReplanFSM::stop(const std::string& reason,bool notify) {
  if (stop_latched_) return;
  if(alignment_active_)publishStartAlignment(false,true);alignment_active_=false;
  stop_latched_=true;waiting_=have_target_;
  if (have_odom_) { manager_->EmergencyStop(position_);publishTrajectory(have_target_); }
  if (notify) { std_msgs::String msg;msg.data="{\"stamp\":"+std::to_string(ros::Time::now().toSec())+",\"reason\":\""+reason+"\"}";collision_pub_.publish(msg); }
  transition(EMERGENCY,reason.c_str());
}
void SCANReplanFSM::execute(const ros::TimerEvent&) {
  updateClock();if (!have_target_ || stop_latched_ || !freshOdom()) return;
  if (state_==ALIGN_START) { updateStartAlignment();return; }
  if (state_==GENERATE) {
    if(beginStartAlignment())return;
    if (plan(false,failures_>0)) transition(EXECUTE,"native rebound trajectory accepted");
    else {
      ++failures_;
      if (!waiting_) stop("native initialization failed");
      else {
        retry_started_=ros::Time::now();retry_at_=retry_started_+ros::Duration(.25);
        retry_cloud_stamp_=manager_->grid_map_->cloudUpdateStamp();
        transition(EMERGENCY,"waiting for a feasible native trajectory");
      }
    }
  } else if (state_==REPLAN) {
    if (replan()) transition(EXECUTE,"native replan accepted");
    else {
      ++failures_;
      // Keep a still-valid old trajectory. The predictive collision callback
      // is the only authority that turns geometric blockage into an emergency.
      const auto& info=manager_->local_data_;
      if ((ros::Time::now()-info.start_time_).toSec()>=info.duration_) stop("local segment exhausted without a successor");
      else transition(EXECUTE,"replan failed; retaining native valid horizon");
    }
  } else if (state_==EMERGENCY) {
    const auto now=ros::Time::now();
    const bool fresh_update=manager_->grid_map_->hasFusedCloud()&&
        manager_->grid_map_->cloudUpdateStamp()>retry_cloud_stamp_&&
        (now-retry_started_).toSec()>=.1;
    if (fail_safe_ && waiting_ && (now>=retry_at_||fresh_update))
      transition(GENERATE,fresh_update?"retry with fresh obstacle map":"retry after confirmed hardware stop");
  } else if (state_==EXECUTE) {
    auto& info=manager_->local_data_;const double elapsed=(ros::Time::now()-info.start_time_).toSec();
    const auto end=route_.at(route_.length());
    if ((end-position_).head<2>().norm()<.20 && std::abs(end.z()-position_.z())<.20) return;
    if (elapsed>=info.duration_-.01 || ((position_-info.start_pos_).norm()>=replan_distance_ && (end-position_).norm()>no_replan_distance_)) transition(REPLAN,"native receding horizon");
  }
}
void SCANReplanFSM::collision(const ros::TimerEvent&) {
  updateClock();
  if (!have_target_||stop_latched_||!freshOdom()||(state_!=EXECUTE&&state_!=REPLAN)) return;
  auto& info=manager_->local_data_;
  const double current=collision_progress_.projectTime(position_);
  if (info.start_time_.isZero()||current>=info.duration_) return;
  const double speed=velocity_.norm(),acceleration=manager_->pp_.max_acc_;
  const double map_age=(ros::Time::now()-manager_->grid_map_->cloudUpdateStamp()).toSec();
  braking_distance_=collisionStopDistance(speed,acceleration,map_age,manager_->grid_map_->getResolution(),emergency_time_);
  collision_distance_=-1.;double distance=0.;Eigen::Vector3d previous=info.position_traj_.evaluateDeBoorT(current);
  // A geometric conflict requests a successor first. A valid successor is
  // handed off without StopMove; brake only when no feasible successor was
  // found and the old curve no longer provides enough collision-free travel.
  const int samples=std::max(1,int(std::ceil((info.duration_-current)/.01)));
  for (int sample=0;sample<=samples;++sample) {
    const double t=current+(info.duration_-current)*sample/samples;
    const auto p=info.position_traj_.evaluateDeBoorT(t);
    distance+=(p-previous).norm();previous=p;
    if(distance>std::max(horizon_,braking_distance_+.1))break;
    const auto next=info.position_traj_.evaluateDeBoorT(std::min(t+.01,info.duration_));
    const Eigen::Vector3d d=next-p;const double heading=d.head<2>().norm()>1e-6?std::atan2(d.y(),d.x()):yaw();
    if (manager_->grid_map_->getInflateOccupancy(p,heading)==0) continue;
    collision_distance_=distance;
    const auto begun=ros::SteadyTime::now();
    if (replan()) transition(EXECUTE,"native collision replan succeeded");
    else {
      const double elapsed=(ros::SteadyTime::now()-begun).toSec();
      if(distance<=braking_distance_+speed*elapsed+.5*acceleration*elapsed*elapsed)
        stop("no feasible collision-free successor inside braking distance");
      else transition(REPLAN,"native future collision needs replanning");
    }
    return;
  }
}
}
