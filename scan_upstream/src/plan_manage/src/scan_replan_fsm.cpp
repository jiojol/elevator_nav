
#include <plan_manage/scan_replan_fsm.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <plan_manage/heading_preview.hpp>
#include <cstdlib>

namespace
{
  constexpr double replan_retry_period = 0.15;

  std::string shellQuote(const std::string &value)
  {
    std::string quoted = "'";
    for (const char c : value)
    {
      if (c == '\'')
        quoted += "'\\''";
      else
        quoted += c;
    }
    quoted += "'";
    return quoted;
  }
} // namespace

namespace scan_upstream_planner
{

  void SCANReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = ros::Time::now();

    ros::param::param("/closed_loop_controller/time_forward", turn_lookahead_, 0.8);
    ros::param::param("/closed_loop_controller/heading_error_threshold", turn_threshold_, 0.8);
    ros::param::param("/closed_loop_controller/kp_yaw", turn_kp_, 1.5);
    ros::param::param("/closed_loop_controller/max_vyaw", turn_rate_, 1.0);
    turn_rate_ = std::clamp(turn_rate_, 0.0, 1.0);  // Controller hard limit.

    /*  fsm param  */
    nh.param("fsm/navi_mode", navi_mode_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/experimental_prefix_cruise_speed", prefix_cruise_speed_, 0.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizon_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/max_replan_fail_count", max_replan_fail_count_, 1000);
    nh.param("grid_map/obstacles_inflation_z_up", self_inflation_z_up_, 0.0);
    nh.param("grid_map/obstacles_inflation_z_down", self_inflation_z_down_, 0.0);
    nh.param("grid_map/double_cylinder_radius", self_double_cylinder_radius_, 0.0);
    nh.param("grid_map/double_cylinder_offset", self_double_cylinder_offset_, 0.0);
    nh.param("grid_map/body_height", body_height_, 0.0);
    nh.param("grid_map/frame_id", self_inflation_frame_id_, std::string("world"));

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const std::string keypoints_yaml = "\"$(rospack find scan_upstream_planner)/../../../tools/keypoint.yaml\"";
      const std::string load_keypoints_cmd =
          "rosparam load " + keypoints_yaml + " " + shellQuote(nh.getNamespace());
      if (std::system(load_keypoints_cmd.c_str()) != 0)
      {
        ROS_ERROR("[SCANReplanFSM] Failed to load keypoints_yaml: tools/keypoint.yaml");
        ros::shutdown();
        return;
      }

      nh.param("fsm/waypoint_num", waypoint_num_, -1);

      if (waypoint_num_ <= 0)
      {
        ROS_ERROR("[SCANReplanFSM] navi_mode=2 requires keypoints_yaml with fsm/waypoint_num and fsm/waypoint{i}_{x,y,z}.");
        ros::shutdown();
        return;
      }
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        nh.param("fsm/waypoint" + to_string(i) + "_x", preset_waypoints_[i](0), -1.0);
        nh.param("fsm/waypoint" + to_string(i) + "_y", preset_waypoints_[i](1), -1.0);
        nh.param("fsm/waypoint" + to_string(i) + "_z", preset_waypoints_[i](2), -1.0);
      }
    }

    XmlRpc::XmlRpcValue elevator;
    if (nh.getParam("fsm/elevator", elevator) && elevator.hasMember("align_exit_yaw") &&
        static_cast<bool>(elevator["align_exit_yaw"]) && elevator.hasMember("floors"))
    {
      auto &stops = elevator["floors"];
      for (auto it = stops.begin(); it != stops.end(); ++it)
      {
        auto &pose = it->second["inside_pose"];
        auto number = [](XmlRpc::XmlRpcValue &v) {
          return v.getType() == XmlRpc::XmlRpcValue::TypeInt ?
              static_cast<double>(static_cast<int>(v)) : static_cast<double>(v);
        };
        elevator_cabins_.emplace_back(number(pose[0]), number(pose[1]), number(pose[2]));
      }
    }
    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &SCANReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &SCANReplanFSM::checkCollisionCallback, this);

    std::string body_pose_topic;
    ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
    odom_sub_ = nh.subscribe(body_pose_topic, 1, &SCANReplanFSM::odometryCallback, this);
    go2_execution_frozen_sub_ = nh.subscribe("/planning/go2_execution_frozen", 10, &SCANReplanFSM::go2ExecutionFrozenCallback, this);

    controller_progress_sub_ = nh.subscribe("/planning/go2_execution_progress", 1,
                                           &SCANReplanFSM::executionProgressCallback, this);

    ros::param::param("/integration/go2_rl", rl_waiting_for_odom_, false);
    if (rl_waiting_for_odom_)
      rl_handoff_sub_ = nh.subscribe("/go2_rl_nav/handoff", 1, &SCANReplanFSM::rlHandoffCallback, this);

    bspline_pub_ = nh.advertise<scan_upstream_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<scan_upstream_planner::DataDisp>("/planning/data_display", 100);
    self_inflation_pub_ = nh.advertise<visualization_msgs::Marker>("self_inflation", 10, true);

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &SCANReplanFSM::rvizGoalCallback, this);
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      planGlobalTrajbyGivenWps();
    }
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      path_sub_ = nh.subscribe("/initial_path", 1, &SCANReplanFSM::pathCallback, this);
      goal_reached_sub_ = nh.subscribe("/planning/goal_reached", 10, &SCANReplanFSM::goalReachedCallback, this);
      ros::param::param("/integration/marked_route", marked_route_, false);
      if (marked_route_)
        marked_turn_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/planning/marked_final_turn", 1);
      free_map_sub_ = nh.subscribe("/plan_map/free", 1, &SCANReplanFSM::freeMapCallback, this);
    }
    else
      cout << "Wrong navi_mode_ value! navi_mode_=" << navi_mode_ << endl;
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory to first preset waypoint!");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      ROS_WARN("[SCANReplanFSM] Ignore RViz goal before receiving initial body pose.");
      return;
    }

    nav_msgs::PathPtr path(new nav_msgs::Path);
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[waypointCallback] Empty waypoint message, ignore.");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.size() < 2)
    {
      ROS_WARN("[planGlobalTrajByWaypoints] Reference path requires at least two points.");
      return false;
    }

    end_pt_ = waypoints.back();
    std::vector<Eigen::Vector3d> reference_waypoints(waypoints.begin() + 1, waypoints.end());

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        waypoints.front(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        reference_waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      ROS_ERROR("Unable to generate global trajectory from waypoints!");
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      ROS_WARN("[navi_mode=%d] No active waypoint to plan.", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      ROS_ERROR("[navi_mode=%d] Unable to generate trajectory to waypoint %d.", navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    ROS_INFO("[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f].",
             navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        ROS_WARN("[global target] Target [%.2f, %.2f, %.2f] is occupied; use backward collision-free point [%.2f, %.2f, %.2f].",
                 raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    ROS_ERROR("[global target] Target is occupied, and no collision-free point was found along the global trajectory.");
    return false;
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      pending_reference_path_.reset();
      if (marked_route_ && reference_goal_active_ && msg && msg->header.stamp >= reference_path_header_.stamp)
      {
        const std_msgs::HeaderConstPtr header(new std_msgs::Header(reference_path_header_));
        goalReachedCallback(header);
      }
      return;
    }

    if (!msg->header.stamp.isZero() && msg->header.stamp <= reference_path_header_.stamp)
      return;
    if (!freshReferenceInputs())
    {
      if (!pending_reference_path_ || msg->header.stamp >= pending_reference_path_->header.stamp)
        pending_reference_path_ = msg;
      ROS_INFO_THROTTLE(2.0, "SCAN: waiting for fresh pose and occupancy before installing the reference path");
      return;
    }
    pending_reference_path_.reset();

    trigger_ = true;
    ingress_direction_.setZero();
    const auto &ingress_pose = msg->poses.back().pose;
    const Eigen::Vector3d endpoint(ingress_pose.position.x, ingress_pose.position.y, ingress_pose.position.z);
    for (const auto &cabin : elevator_cabins_)
    {
      if ((endpoint - cabin).norm() < .03)
      {
        const auto &q = ingress_pose.orientation;
        const double yaw = std::atan2(2. * (q.w*q.z + q.x*q.y), 1. - 2. * (q.y*q.y + q.z*q.z));
        ingress_direction_ = Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
        break;
      }
    }
    end_pt_ << msg->poses.back().pose.position.x,
        msg->poses.back().pose.position.y,
        msg->poses.back().pose.position.z + body_height_;

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());
    constexpr double min_dist = 0.5;
    Eigen::Vector3d last_wp;
    bool first = true;

    for (const auto &pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = pose_stamped.pose.position.z + body_height_;

      if (first || (wp - last_wp).norm() >= min_dist)
      {
        waypoints.push_back(wp);
        last_wp = wp;
        first = false;
      }
    }

    if ((waypoints.back() - end_pt_).norm() > 1e-6)
      waypoints.push_back(end_pt_);

    bool at_mark = false;
    const Eigen::Vector3d goal = end_pt_;
    double goal_yaw = 0.0;
    if (marked_route_)
    {
      const auto &q = msg->poses.back().pose.orientation;
      const Eigen::Quaterniond goal_orientation(q.w, q.x, q.y, q.z);
      if (!goal_orientation.coeffs().allFinite() || goal_orientation.norm() < 1e-9)
        return;
      const auto rotation = goal_orientation.normalized().toRotationMatrix();
      goal_yaw = std::atan2(rotation(1, 0), rotation(0, 0));
      const Eigen::Vector3d delta = odom_pos_ - goal;
      at_mark = delta.head<2>().norm() <= 0.15 && std::abs(delta.z()) <= 0.18;
    }

    const bool success = at_mark || planGlobalTrajByWaypoints(waypoints);
    if (success)
    {
      reference_path_header_ = msg->header;
      reference_goal_active_ = true;
      if (marked_route_)
      {
        marked_goal_ = goal;
        marked_goal_yaw_ = goal_yaw;
      }
      stopped_conflict_.reset();
      reference_route_.clear();
      reference_distance_.clear();
      last_failed_full_replan_ = ros::Time();
      for (const auto &pose : msg->poses)
      {
        const auto &pt = pose.pose.position;
        const Eigen::Vector3d position(pt.x, pt.y, pt.z + body_height_);
        const double step = reference_route_.empty() ? 0.0 : (position - reference_route_.back()).norm();
        if (reference_route_.empty() || step > 1e-6)
        {
          reference_route_.push_back(position);
          reference_distance_.push_back(reference_distance_.empty() ? 0.0 : reference_distance_.back() + step);
        }
      }
      /*** FSM ***/
      if (at_mark)
      {
        trigger_ = have_target_ = have_new_target_ = false;
        changeFSMExecState(WAIT_TARGET, "MARKED_POSITION_REACHED");
      }
      else if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      ROS_INFO("==========================================\n");
    }
    else
    {
      ROS_ERROR("❌ Unable to generate global trajectory!");
    }
  }

  void SCANReplanFSM::goalReachedCallback(const std_msgs::HeaderConstPtr &msg)
  {
    if (!reference_goal_active_ || msg->stamp.isZero() ||
        msg->stamp != reference_path_header_.stamp || msg->frame_id != reference_path_header_.frame_id)
      return;

    reference_goal_active_ = false;
    trigger_ = have_target_ = have_new_target_ = false;
    flag_escape_emergency_ = need_hover_stop_ = holding_emergency_stop_ = false;
    replan_fail_count_ = 0;
    last_failed_full_replan_ = ros::Time();
    short_replan_time_.reset();
    stopped_conflict_.reset();
    reference_route_.clear();
    reference_distance_.clear();
    // Retire the trajectory without recycling its ID. Queued controller
    // progress for this curve cannot revive it before the next plan.
    auto &trajectory = planner_manager_->local_data_;
    trajectory.start_time_ = ros::Time();
    trajectory.duration_ = 0.0;
    controller_traj_id_ = -1;
    controller_progress_at_ = ros::Time();
    changeFSMExecState(WAIT_TARGET, "GOAL_REACHED");
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && have_target_ &&
        !odom_stamp_.isZero() && (msg->header.stamp - odom_stamp_).toSec() > .5)
      reference_inputs_paused_ = true;
    odom_stamp_ = msg->header.stamp;
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      ROS_INFO("[SCANReplanFSM] Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();
  }

  void SCANReplanFSM::rlHandoffCallback(const std_msgs::TimeConstPtr &msg)
  {
    // Zero pauses the FSM. A nonzero request requires newer measured odometry.
    rl_handoff_stamp_ = msg->data;
    rl_waiting_for_odom_ = true;
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::BoolConstPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::executionProgressCallback(const std_msgs::Float64MultiArrayConstPtr &msg)
  {
    const auto &info = planner_manager_->local_data_;
    if (info.start_time_.isZero() || msg->data.size() != 2 || msg->data[0] != static_cast<double>(info.traj_id_) ||
        !std::isfinite(msg->data[1]) || msg->data[1] < 0.0 ||
        msg->data[1] > info.duration_ + 1e-6)
      return;
    controller_traj_id_ = info.traj_id_;
    controller_traj_time_ = msg->data[1];
    controller_progress_at_ = ros::Time::now();
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const ros::Time now = ros::Time::now();
    double dt = (now - last_freeze_update_time_).toSec();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    const double progress_age = (now - controller_progress_at_).toSec();
    if (controller_traj_id_ == info->traj_id_ && progress_age >= 0.0 && progress_age <= 0.2)
      info->start_time_ = now - ros::Duration(controller_traj_time_);
    else if (go2_execution_frozen_ && info->start_time_.toSec() > 1e-5)
      info->start_time_ += ros::Duration(dt);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    // Reflect obstacle inflation about the query point to display the
    // equivalent body envelope: obstacle-up is body-down, and vice versa.
    const double z_up = std::max(0.0, self_inflation_z_down_);
    const double z_down = std::max(0.0, self_inflation_z_up_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = ros::Duration(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    // Match the measured-body probe axis used by GridMap on stair inclines.
    const Eigen::Vector3d normal = odom_orient_.toRotationMatrix().col(2);
    if (normal.z() > 0.3)
    {
      heading.z() = -normal.head<2>().dot(heading.head<2>()) / normal.z();
      heading.normalize();
    }
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_.publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_.publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  bool SCANReplanFSM::freshReferenceInputs() const
  {
    const auto &map = planner_manager_->grid_map_;
    if (!have_odom_ || !map || odom_stamp_.isZero() || map->cloudStamp().isZero())
      return false;
    const auto now = ros::Time::now();
    const double pose_age = (now - odom_stamp_).toSec();
    const double map_age = (now - map->cloudStamp()).toSec();
    return pose_age >= -.1 && pose_age <= .5 && map_age >= -.1 && map_age <= 1.;
  }

  void SCANReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    if (!collision_stop_at_.isZero())
    {
      const ros::Time now = ros::Time::now();
      const double pose_age = (now - odom_stamp_).toSec();
      const bool waiting_for_cloud = navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
          planner_manager_->grid_map_->cloudStamp() <= collision_stop_at_;
      if ((now - collision_stop_at_).toSec() < .35 || !have_odom_ ||
          pose_age < -.1 || pose_age > .5 || odom_vel_.head<2>().norm() > .10 || waiting_for_cloud)
        return;
      collision_stop_at_ = ros::Time();
    }
    if (rl_waiting_for_odom_)
    {
      if (rl_handoff_stamp_.isZero() || odom_stamp_ <= rl_handoff_stamp_)
        return;
      rl_waiting_for_odom_ = false;
      if (have_target_)
      {
        auto &trajectory = planner_manager_->local_data_;
        trajectory.start_time_ = ros::Time();
        trajectory.duration_ = 0.0;
        controller_traj_id_ = -1;
        controller_progress_at_ = ros::Time();
        holding_emergency_stop_ = need_hover_stop_ = false;
        replan_fail_count_ = 0;
        last_failed_full_replan_ = ros::Time();
        have_new_target_ = true;
        changeFSMExecState(GEN_NEW_TRAJ, "RL_HANDOFF");
      }
    }

    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      if (have_target_ || pending_reference_path_)
      {
        if (!freshReferenceInputs())
        {
          reference_inputs_paused_ = true;
          ROS_WARN_THROTTLE(2., "SCAN planning paused: map pose or lidar occupancy is stale; goal retained");
          return;
        }
        const auto epoch = planner_manager_->grid_map_->alignmentEpoch();
        const bool new_alignment = reference_alignment_epoch_ && epoch != reference_alignment_epoch_;
        reference_alignment_epoch_ = epoch;
        if ((reference_inputs_paused_ || new_alignment) && have_target_)
        {
          auto &trajectory = planner_manager_->local_data_;
          trajectory.start_time_ = ros::Time();
          trajectory.duration_ = 0.0;
          controller_traj_id_ = -1;
          controller_progress_at_ = ros::Time();
          holding_emergency_stop_ = need_hover_stop_ = false;
          stopped_conflict_.reset();
          short_replan_time_.reset();
          replan_fail_count_ = 0;
          last_failed_full_replan_ = ros::Time();
          have_new_target_ = true;
          changeFSMExecState(GEN_NEW_TRAJ, "FRESH_INPUTS");
        }
        reference_inputs_paused_ = false;
        if (pending_reference_path_)
        {
          const auto path = pending_reference_path_;
          pathCallback(path);
        }
      }
    }

    updateLocalTrajTimeFreeze();

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      const auto now = ros::Time::now();
      if (holding_emergency_stop_ && !last_failed_full_replan_.isZero() && now >= last_failed_full_replan_ &&
          (now - last_failed_full_replan_).toSec() < replan_retry_period)
        break;
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      const bool success = ((holding_emergency_stop_ || !ingress_direction_.isZero()) && tryShortForwardPlan()) ||
                           callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        if (holding_emergency_stop_)
          last_failed_full_replan_ = now;
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      // Retry the normal horizon at a bounded rate while following the current
      // segment. A verified shorter segment can hand off to another short one.
      const auto now = ros::Time::now();
      if (!last_failed_full_replan_.isZero() && now >= last_failed_full_replan_ &&
          (now - last_failed_full_replan_).toSec() < replan_retry_period)
        break;

      const bool full_plan = planFromCurrentTraj();
      if (!full_plan)
        last_failed_full_replan_ = now;

      if (full_plan || tryShortForwardPlan())
      {
        last_failed_full_replan_ = ros::Time();
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);
      if (short_replan_time_ && t_cur >= *short_replan_time_)
      {
        changeFSMExecState(REPLAN_TRAJ, "SHORT_SEGMENT");
        return;
      }

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(std::max(0.0, t_cur));

      // If the measured body has moved beyond the current segment, replan
      // from odometry before the old swept path can become the stop trigger.
      constexpr double reanchor_tracking_gap = 0.45;
      if (navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
          (pos - odom_pos_).head<2>().norm() > reanchor_tracking_gap)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRACKING_GAP");
        return;
      }

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_)
        {
          // A stop command does not guarantee that estimated velocity becomes
          // zero (stair slip, tracking lag, or recorded feedback). Keep solving
          // from fresh measured state; GEN_NEW_TRAJ rate-limits retries and
          // retains the stop until a collision/dynamics-checked curve commits.
          const double pose_age = (ros::Time::now() - odom_stamp_).toSec();
          if (have_odom_ && pose_age >= -0.1 && pose_age <= 0.5)
            changeFSMExecState(GEN_NEW_TRAJ, "STOP_RETRY");
        }
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          ROS_INFO("Exiting EMERGENCY_STOP. Switching to WAIT_TARGET. Need a new target point.");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      ROS_WARN("Replan failed %d times. Emergency stop and wait for a new target.", replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      const bool previous_finished = t_cur >= info->duration_ - 1e-3;
      if (!previous_finished)
        start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      constexpr double max_tracking_gap_xy = 0.25;
      const bool reanchor = previous_finished || (start_pt_ - odom_pos_).head<2>().norm() > max_tracking_gap_xy;
      if (reanchor)
      {
        start_pt_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();
      }
      else
      {
        start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
        start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
      }

      bool success = callReboundReplan(reanchor, false);
      if (!success && !reanchor)
        success = callReboundReplan(true, false);
      if (!success)
        success = callReboundReplan(true, true);
      return success;
    }

    start_pt_ = odom_pos_;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    if (!planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      ROS_ERROR("[navi_mode=%d] Unable to refresh global trajectory from odom to current target.", navi_mode_);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    if (holding_emergency_stop_)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.toSec() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (ros::Time::now() - info->start_time_).toSec();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::publishMarkedTurn()
  {
    if (!marked_route_ || !reference_goal_active_ || !freshReferenceInputs() ||
        odom_vel_.norm() >= 0.08)
      return;
    const Eigen::Vector3d displacement = odom_pos_ - marked_goal_;
    if (displacement.head<2>().norm() > 0.15 || std::abs(displacement.z()) > 0.18)
      return;
    // Align the final heading only on a level stance. Preserve the swept
    // footprint check for the complete remaining turn plus a small overrun.
    if (odom_orient_.toRotationMatrix()(2, 2) < std::cos(12.0 * M_PI / 180.0))
      return;
    const double yaw = getOdomYaw();
    const double error = std::remainder(marked_goal_yaw_ - yaw, 2.0 * M_PI);
    if (std::abs(error) <= 8.0 * M_PI / 180.0)
      return;
    const double sweep = error + std::copysign(0.08, error);
    const int steps = static_cast<int>(std::ceil(std::abs(sweep) / 0.05));
    const auto &map = planner_manager_->grid_map_;
    for (int step = 0; step <= steps; ++step)
    {
      const double heading = yaw + sweep * static_cast<double>(step) / steps;
      if (map->getInflateOccupancy(odom_pos_, heading) != 0 ||
          !bodyInsideFree(odom_pos_, heading, marked_goal_.z() - body_height_))
      {
        geometry_msgs::TwistStamped hold;
        hold.header = reference_path_header_;
        marked_turn_pub_.publish(hold);
        return;
      }
    }
    geometry_msgs::TwistStamped turn;
    turn.header = reference_path_header_;
    const double limit = std::min(turn_rate_, 0.35);
    turn.twist.angular.z = std::clamp(error, -limit, limit);
    marked_turn_pub_.publish(turn);
  }

  void SCANReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    publishMarkedTurn();
    if (rl_waiting_for_odom_)
      return;
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
        (reference_inputs_paused_ || !freshReferenceInputs() ||
         planner_manager_->grid_map_->alignmentEpoch() != reference_alignment_epoch_))
      return;
    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    // Recovery candidates are checked by the planner. A stationary stop
    // spline must never trigger another collision-driven replan.
    if (exec_state_ == WAIT_TARGET || exec_state_ == EMERGENCY_STOP ||
        holding_emergency_stop_ || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      const int occupancy = map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next));
      if (occupancy != 0)
      {
        if (t - t_cur < emergency_time_)
        {
          ROS_WARN("Predicted trajectory conflict. emergency stop: ahead=%.3fs occupancy=%d "
                   "query=(%.3f, %.3f, %.3f) measured_body=(%.3f, %.3f, %.3f)",
                   t - t_cur, occupancy, pos.x(), pos.y(), pos.z(), odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
          stopped_conflict_ = pos;
          collision_stop_at_ = ros::Time::now();
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          callEmergencyStop(odom_pos_);
          flag_escape_emergency_ = false;
        }
        else if (planFromCurrentTraj())
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
        else
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        return;
      }
    }
  }

  double SCANReplanFSM::referenceProgress(const Eigen::Vector3d &position,
                                          double lower, double upper) const
  {
    double best_distance = std::numeric_limits<double>::infinity();
    double best_progress = lower;
    for (size_t i = 1; i < reference_route_.size(); ++i)
    {
      const double begin = reference_distance_[i - 1];
      const double end = reference_distance_[i];
      if (end < lower || begin > upper)
        continue;

      const Eigen::Vector3d segment = reference_route_[i] - reference_route_[i - 1];
      const double length = end - begin;
      const double along = std::clamp(segment.dot(position - reference_route_[i - 1]) / length,
                                      std::max(0.0, lower - begin), std::min(length, upper - begin));
      const double error = (position - (reference_route_[i - 1] + along / length * segment)).squaredNorm();
      if (error < best_distance)
      {
        best_distance = error;
        best_progress = begin + along;
      }
    }
    return best_progress;
  }

  bool SCANReplanFSM::followsStairProgress(UniformBspline &candidate, bool &stair) const
  {
    stair = false;
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH || reference_distance_.size() < 2)
      return true;

    const double total = reference_distance_.back();
    const double body_progress = referenceProgress(odom_pos_, 0.0, total);
    const double lower = std::max(0.0, body_progress - 1.0);
    const double upper = std::min(total, body_progress + planning_horizon_ + 1.0);
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -min_z;
    for (size_t i = 0; i < reference_route_.size(); ++i)
    {
      if (reference_distance_[i] < lower || reference_distance_[i] > upper)
        continue;
      min_z = std::min(min_z, reference_route_[i].z());
      max_z = std::max(max_z, reference_route_[i].z());
    }
    constexpr double stair_elevation_span = 0.30;
    if (max_z - min_z < stair_elevation_span)
      return true;
    stair = true;

    const Eigen::Vector3d initial = candidate.evaluateDeBoorT(0.0);
    const double duration = candidate.getTimeSum();
    const Eigen::Vector2d body_heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()));
    const Eigen::Vector2d route_heading =
        (referencePosition(std::min(total, body_progress + 0.35)) -
         referencePosition(body_progress)).head<2>();
    double lookahead = 0.8;
    ros::param::getCached("/closed_loop_controller/time_forward", lookahead);
    const Eigen::Vector2d departure = headingPreview(candidate, 0.0, duration, lookahead, initial.head<2>());
    // If the body already faces up or down this flight, don't hand off to a
    // spline that would require spinning around just to begin moving.
    if (route_heading.norm() > 0.05 && !departure.isZero(1e-9) &&
        body_heading.dot(route_heading.normalized()) > std::cos(0.8) &&
        departure.dot(route_heading) < 0.0)
      return false;

    // A short reversal can be below the arc-length tolerance and still
    // demand a large turn in the next control update. Only apply the tighter
    // heading check on the flight, so turns on a level landing remain possible.
    const bool aligned_with_flight = route_heading.norm() > 0.05 &&
        body_heading.dot(route_heading.normalized()) > std::cos(0.8);
    if (aligned_with_flight)
    {
      const double preview_end = std::min(2.0, duration);
      for (double t = 0.0; t <= preview_end; t += 0.1)
      {
        const Eigen::Vector3d position = candidate.evaluateDeBoorT(t);
        const double progress = referenceProgress(position, lower, upper);
        const Eigen::Vector3d guide = referencePosition(progress + 0.30) -
                                     referencePosition(progress - 0.30);
        if (std::abs(guide.z()) < 0.09 || guide.head<2>().norm() < 0.10)
          continue;
        const Eigen::Vector2d direction = headingPreview(candidate, t, duration, lookahead, position.head<2>());
        if (!direction.isZero(1e-9) && direction.normalized().dot(guide.head<2>().normalized()) < std::cos(1.1))
          return false;
      }
    }

    // Compare arc length along the reference rather than a fixed XY bearing,
    // so climbing the next flight after a landing is still forward progress.
    constexpr double allowed_reverse = 0.20;
    double furthest = body_progress;
    const int samples = std::max(1, static_cast<int>(std::ceil(duration / 0.05)));
    for (int i = 0; i <= samples; ++i)
    {
      const double time = duration * i / samples;
      const double progress = referenceProgress(candidate.evaluateDeBoorT(time), lower, upper);
      if (furthest - progress > allowed_reverse)
        return false;
      furthest = std::max(furthest, progress);
    }
    return true;
  }

  Eigen::Vector3d SCANReplanFSM::referencePosition(double progress) const
  {
    auto hi = std::upper_bound(reference_distance_.begin(), reference_distance_.end(), progress);
    if (hi == reference_distance_.begin())
      return reference_route_.front();
    if (hi == reference_distance_.end())
      return reference_route_.back();

    const auto i = static_cast<size_t>(hi - reference_distance_.begin());
    const double fraction = (progress - reference_distance_[i - 1]) /
                            (reference_distance_[i] - reference_distance_[i - 1]);
    return (1.0 - fraction) * reference_route_[i - 1] + fraction * reference_route_[i];
  }

  // Re-seed along this flight with the original endpoints. Preserve the
  // polynomial seed's arc-length fractions so its acceleration ramp is kept.
  void SCANReplanFSM::shapeForwardSeed(std::vector<Eigen::Vector3d> &seed) const
  {
    if (seed.size() < 3)
      return;
    const double total = reference_distance_.back();
    const double start = referenceProgress(seed.front(), 0.0, total);
    const double end = referenceProgress(seed.back(), start, std::min(total, start + planning_horizon_ + 1.0));
    std::vector<double> arc(seed.size(), 0.0);
    for (size_t i = 1; i < seed.size(); ++i)
      arc[i] = arc[i - 1] + (seed[i] - seed[i - 1]).norm();
    if (arc.back() < 0.1 || end <= start)
      return;
    const Eigen::Vector2d entry = (seed.front() - referencePosition(start)).head<2>();
    const Eigen::Vector2d exit = (seed.back() - referencePosition(end)).head<2>();
    for (size_t i = 1; i + 1 < seed.size(); ++i)
    {
      const double u = arc[i] / arc.back();
      seed[i].head<2>() = referencePosition(start + u * (end - start)).head<2>() +
                          (1.0 - u) * entry + u * exit;
    }
  }

  void SCANReplanFSM::shapeReferenceHeight(std::vector<Eigen::Vector3d> &seed) const
  {
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH || reference_distance_.size() < 2 || seed.size() < 3)
      return;

    const double total = reference_distance_.back();
    const double start = referenceProgress(start_pt_, 0.0, total);
    const double last = referenceProgress(local_target_pt_, start,
                                          std::min(total, start + planning_horizon_ + 1.0));
    if (last - start < 0.20 || std::abs(referencePosition(last).z() - referencePosition(start).z()) < 0.20)
      return;

    const auto height = [this](double progress) {
      // Round individual voxel height jumps while keeping the landing and
      // stair slope where they occur along the route.
      return 0.25 * referencePosition(progress - 0.15).z() + 0.50 * referencePosition(progress).z() +
             0.25 * referencePosition(progress + 0.15).z();
    };
    const double initial_offset = start_pt_.z() - height(start);
    const double terminal_offset = local_target_pt_.z() - height(last);
    for (size_t i = 1; i + 1 < seed.size(); ++i)
    {
      const double progress = referenceProgress(seed[i], start, last);
      const double fraction = (progress - start) / (last - start);
      seed[i].z() = height(progress) + (1.0 - fraction) * initial_offset + fraction * terminal_offset;
    }
  }

  void SCANReplanFSM::freeMapCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    double cell_size;
    if (msg->header.frame_id != "map" || msg->is_bigendian ||
        !ros::param::getCached("/plan_map/resolution", cell_size) ||
        !std::isfinite(cell_size) || cell_size < 0.02 || cell_size > 0.5)
      return;
    if (free_cell_size_ == cell_size && free_width_ == msg->width &&
        free_row_step_ == msg->row_step && free_cloud_bytes_ == msg->data)
      return;

    std::set<std::array<int, 3>> cells;
    try
    {
      for (sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
           x != x.end(); ++x, ++y, ++z)
      {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z))
          cells.insert({int(std::floor(*x / cell_size)), int(std::floor(*y / cell_size)),
                        int(std::floor(*z / cell_size))});
      }
    }
    catch (const std::runtime_error &error)
    {
      ROS_WARN_THROTTLE(2.0, "Cannot read FREE map for lateral candidates: %s", error.what());
      free_cells_.clear();
      return;
    }
    free_cells_ = std::move(cells);
    free_cell_size_ = cell_size;
    free_width_ = msg->width;
    free_row_step_ = msg->row_step;
    free_cloud_bytes_ = msg->data;
  }

  bool SCANReplanFSM::bodyInsideFree(const Eigen::Vector3d &position, double yaw, double floor_z) const
  {
    if (free_cells_.empty())
      return false;

    const Eigen::Vector2d heading(std::cos(yaw), std::sin(yaw));
    const int iz = int(std::floor(floor_z / free_cell_size_));
    // Use the reference stair layer for terrain coverage, not the model Z.
    // At a stair edge an adjacent height cell may be the actual tread.
    for (double offset : {-self_double_cylinder_offset_, 0.0, self_double_cylinder_offset_})
    {
      const Eigen::Vector2d p = position.head<2>() + offset * heading;
      const int ix = int(std::floor(p.x() / free_cell_size_));
      const int iy = int(std::floor(p.y() / free_cell_size_));
      if (!free_cells_.count({ix, iy, iz}) && !free_cells_.count({ix, iy, iz - 1}) &&
          !free_cells_.count({ix, iy, iz + 1}))
        return false;
    }
    return true;
  }

  double SCANReplanFSM::shortReplanTime(UniformBspline &curve)
  {
    const double duration = curve.getTimeSum();
    const double halfway = 0.5 * duration;
    const double latest = duration - 3.0 * replan_retry_period;
    const Eigen::Vector2d chord =
        (curve.evaluateDeBoorT(duration) - curve.evaluateDeBoorT(0.0)).head<2>();
    if (latest <= halfway || chord.norm() < 0.05)
      return halfway;

    // Let the recovery maneuver make progress first. Defer its usual
    // handoff only if the checked suffix has a faster forward interval.
    const Eigen::Vector2d forward = chord.normalized();
    const auto forward_speed = [&](double t) {
      const Eigen::Vector2d step =
          (curve.evaluateDeBoorT(t + replan_retry_period) - curve.evaluateDeBoorT(t)).head<2>();
      return forward.dot(step) / replan_retry_period;
    };
    const double initial_speed = forward_speed(halfway);
    const int samples = std::clamp(static_cast<int>(std::ceil((latest - halfway) / 0.05)), 1, 128);
    double best_time = halfway, best_speed = initial_speed;
    for (int i = 1; i <= samples; ++i)
    {
      const double t = halfway + (latest - halfway) * i / samples;
      const double speed = forward_speed(t);
      if (speed > best_speed)
      {
        best_time = t;
        best_speed = speed;
      }
    }
    return best_speed > std::max(0.0, initial_speed) + 0.05 ? best_time : halfway;
  }

  bool SCANReplanFSM::tryShortForwardPlan()
  {
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH || reference_distance_.size() < 2)
      return false;

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();
    double preferred_side = 0.0;
    if (holding_emergency_stop_ && stopped_conflict_)
    {
      const double total = reference_distance_.back();
      const double begin = referenceProgress(odom_pos_, 0.0, total);
      const double conflict = referenceProgress(*stopped_conflict_, std::max(0.0, begin - 0.5),
                                                std::min(total, begin + planning_horizon_));
      const Eigen::Vector2d along =
          (referencePosition(conflict + 0.20) - referencePosition(conflict - 0.20)).head<2>();
      const Eigen::Vector2d offset = (*stopped_conflict_ - referencePosition(conflict)).head<2>();
      const double side = along.x() * offset.y() - along.y() * offset.x();
      if (along.norm() > 1e-6 && std::abs(side / along.norm()) >= 0.05)
        preferred_side = side < 0.0 ? 1.0 : -1.0;
    }

    // Keep the collision-guided candidates and add a checked, centered
    // reference candidate if none has a usable endpoint around the landing.
    if (preferred_side != 0.0 && !free_cells_.empty())
      for (double horizon : {1.2, 0.9, 0.55, 0.35})
        for (double offset : {0.26, 0.16})
          if (callReboundReplan(true, false, horizon, preferred_side * offset))
            return true;

    for (double horizon : {0.9, 0.55, 0.35})
    {
      if (callReboundReplan(true, false, horizon))
        return true;
      if (free_cells_.empty())
        continue;
      for (double lateral : {0.16, -0.16, 0.26, -0.26})
        if (preferred_side * lateral <= 0.0 && callReboundReplan(true, false, horizon, lateral))
          return true;
    }

    for (double horizon : {0.70, 0.42})
      if (callReboundReplan(true, false, horizon, 0.0, true))
        return true;
    return false;
  }

  bool SCANReplanFSM::handoffClear(UniformBspline &candidate, bool entire_curve) const
  {
    const auto map = planner_manager_->grid_map_;
    const double yaw = getOdomYaw();
    const Eigen::Vector3d start = candidate.evaluateDeBoorT(0.0);
    const Eigen::Vector3d gap = start - odom_pos_;
    const int steps = std::max(1, static_cast<int>(std::ceil(gap.norm() / (0.5 * map->getResolution()))));
    for (int step = 0; step <= steps; ++step)
      if (map->getInflateOccupancy(odom_pos_ + gap * (static_cast<double>(step) / steps), yaw) != 0)
        return false;

    const auto clear = [&](const Eigen::Vector3d &point, double heading) {
      return map->getInflateOccupancy(point, heading) == 0;
    };
    if (!previewTurns(candidate, yaw, turn_lookahead_, turn_threshold_, turn_kp_, turn_rate_, clear).clear)
      return false;

    // The execution collision timer also queries the trajectory tangent.
    // Check its first normal handoff interval with that same footprint.
    const double duration = candidate.getTimeSum();
    const int samples = std::max(1, static_cast<int>(std::ceil(duration / 0.01)));
    for (int i = 0; i <= samples; ++i)
    {
      const double t = duration * i / samples;
      const Eigen::Vector3d pos = candidate.evaluateDeBoorT(t);
      const Eigen::Vector3d next = candidate.evaluateDeBoorT(std::min(t + 0.01, duration));
      if (!clear(pos, estimateYawFromSegment(pos, next)))
        return false;
      if (!entire_curve && (pos - start).norm() >= replan_thresh_)
        break;
    }
    return true;
  }

  bool SCANReplanFSM::shortTrajectoryClear(UniformBspline &candidate, bool check_free_band) const
  {
    const auto map = planner_manager_->grid_map_;
    const double duration = candidate.getTimeSum();
    const double total = reference_distance_.back();
    const double origin = referenceProgress(odom_pos_, 0.0, total);
    const double lower = std::max(0.0, origin - 1.0);
    const double upper = std::min(total, origin + planning_horizon_ + 1.0);
    if (referenceProgress(candidate.evaluateDeBoorT(duration), lower, upper) - origin < 0.20)
      return false;

    const int samples = std::max(1, static_cast<int>(std::ceil(duration / 0.025)));
    for (int i = 0; i <= samples; ++i)
    {
      const double t = duration * i / samples;
      const double before = std::max(0.0, t - 0.025);
      const double after = std::min(duration, t + 0.025);
      const Eigen::Vector3d pose = candidate.evaluateDeBoorT(t);
      const double yaw = estimateYawFromSegment(candidate.evaluateDeBoorT(before),
                                                candidate.evaluateDeBoorT(after));
      if (map->getInflateOccupancy(pose, yaw) != 0)
        return false;
      if (check_free_band)
      {
        const double progress = referenceProgress(pose, lower, upper);
        if (!bodyInsideFree(pose, yaw, referencePosition(progress).z() - body_height_))
          return false;
      }
    }
    return true;
  }

  bool SCANReplanFSM::solveLocalCandidate(UniformBspline &out, bool use_poly, bool randomize,
                                        bool forward_seed, bool short_segment, bool lateral, bool &stair)
  {
    return planner_manager_->reboundReplan(
          start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_,
          use_poly, randomize, out, replan_thresh_,
          !short_segment && !holding_emergency_stop_ && ingress_direction_.isZero() &&
              navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
              (local_target_pt_ - end_pt_).norm() > no_replan_thresh_ ? prefix_cruise_speed_ : 0.0,
          [&](std::vector<Eigen::Vector3d> &seed) {
            if (forward_seed)
              shapeForwardSeed(seed);
            shapeReferenceHeight(seed);
          },
          [&](UniformBspline &candidate) {
            if (!followsElevatorIngress(candidate))
            {
              ROS_WARN_THROTTLE(1.0, "Rejecting local trajectory that turns away from the elevator during ingress.");
              return false;
            }
            bool candidate_is_stair = false;
            if (!handoffClear(candidate) || !followsStairProgress(candidate, candidate_is_stair))
              return false;
            if ((forward_seed || short_segment) && !shortTrajectoryClear(candidate, forward_seed || lateral))
              return false;
            stair = candidate_is_stair;
            return true;
          },
          [&](UniformBspline &candidate) { return handoffClear(candidate, true); });
  }

  bool SCANReplanFSM::selectStairCandidate(UniformBspline &selected, double &short_horizon)
  {
    const double yaw = getOdomYaw();
    const auto unobstructed = [&](const Eigen::Vector3d &p, double heading) {
      return planner_manager_->grid_map_->getInflateOccupancy(p, heading) == 0;
    };
    const auto incumbent = previewTurns(selected, yaw, turn_lookahead_, turn_threshold_, turn_kp_,
                                        turn_rate_, unobstructed);
    constexpr double turn_budget = 0.20 + 1e-8;
    const bool incumbent_tracks = incumbent.clear && incumbent.longest_turn <= turn_budget;
    if (incumbent_tracks && short_horizon > 0.0)
      return true;

    const double origin = referenceProgress(odom_pos_, 0.0, reference_distance_.back());
    const double upper = std::min(reference_distance_.back(), origin + planning_horizon_ + 1.0);
    const double lower = std::max(0.0, origin - 1.0);
    const auto progress = [&](const Eigen::Vector3d &p) { return referenceProgress(p, lower, upper); };
    const auto lateral_displacement = [&](const Eigen::Vector3d &p) -> Eigen::Vector2d {
      const double s = progress(p);
      const Eigen::Vector2d difference = (p - referencePosition(s)).head<2>();
      const Eigen::Vector2d tangent = (referencePosition(s + 0.15) - referencePosition(s - 0.15)).head<2>();
      if (tangent.squaredNorm() <= 1e-12)
        return difference;
      const Eigen::Vector2d normal = Eigen::Vector2d(-tangent.y(), tangent.x()).normalized();
      return normal.dot(difference) * normal;
    };
    const auto peak_lateral_error = [&](UniformBspline &curve) {
      const Eigen::Vector3d start = curve.evaluateDeBoorT(0.0);
      const double duration = std::min(4.0, curve.getTimeSum());
      const int samples = std::max(1, static_cast<int>(std::ceil(duration / 0.10)));
      double peak = lateral_displacement(start).norm();
      for (int i = 1; i <= samples; ++i)
      {
        const Eigen::Vector3d p = curve.evaluateDeBoorT(duration * i / samples);
        peak = std::max(peak, lateral_displacement(p).norm());
        // Include the section after a failed normal handoff as drift can grow there.
        if ((p - start).norm() >= 2.0 * replan_thresh_)
          break;
      }
      return peak;
    };
    constexpr double max_centering_error = 0.12; // metres from the guide
    constexpr double min_centering_gain = 0.05;
    const double old_advance = progress(selected.evaluateDeBoorT(incumbent.phase)) - origin;
    if (incumbent_tracks)
    {
      const double old_lateral = peak_lateral_error(selected);
      const Eigen::Vector3d start = selected.evaluateDeBoorT(0.0);
      if (old_lateral < max_centering_error ||
          old_lateral < lateral_displacement(start).norm() + min_centering_gain)
        return true;

      constexpr int degree = 3;
      const Eigen::MatrixXd control_points = selected.getControlPoint();
      const Eigen::VectorXd knots = selected.getKnot();
      Eigen::MatrixXd shifts = Eigen::MatrixXd::Zero(control_points.rows(), control_points.cols());
      // Leave the three boundary controls on either end fixed, preserving
      // endpoint position, velocity and acceleration at the original knot times.
      for (Eigen::Index i = degree; i + degree < control_points.cols(); ++i)
      {
        const double time = knots.segment(i + 1, degree).mean() - knots(degree);
        const Eigen::Vector3d p = selected.evaluateDeBoorT(time);
        const double time_weight = std::clamp((4.6 - time) / 0.6, 0.0, 1.0);
        const double distance_weight = std::clamp((2.0 * replan_thresh_ + 0.30 - (p - start).norm()) / 0.30, 0.0, 1.0);
        const Eigen::Vector2d displacement = lateral_displacement(p);
        const double correction = std::min(0.8, 0.15 / std::max(1e-6, displacement.norm()));
        shifts.col(i).head<2>() = -time_weight * distance_weight * correction * displacement;
      }
      for (double weight : {1.0, 0.5})
      {
        UniformBspline alternate(control_points + weight * shifts, degree, selected.getInterval());
        alternate.setKnot(knots);
        const double new_lateral = peak_lateral_error(alternate);
        if (new_lateral > old_lateral - min_centering_gain)
          continue;

        const auto preview = previewTurns(alternate, yaw, turn_lookahead_, turn_threshold_, turn_kp_,
                                          turn_rate_, unobstructed);
        const double advance = progress(alternate.evaluateDeBoorT(preview.phase)) - origin;
        constexpr double advance_tolerance = 0.03;
        if (!preview.clear || preview.longest_turn > turn_budget ||
            advance < std::max(0.05, old_advance - advance_tolerance))
          continue;
        bool stair = false;
        if (!shortTrajectoryClear(alternate, true) || !handoffClear(alternate) ||
            !followsStairProgress(alternate, stair) || !planner_manager_->checkDynamicFeasibility(alternate))
          continue;

        selected = alternate;
        ROS_INFO("Stair centering: original peak=%.3f m, selected peak=%.3f m, selected=forward",
                 old_lateral, new_lateral);
        return true;
      }
      ROS_INFO("Stair centering: original peak=%.3f m, selected peak=%.3f m, selected=original",
               old_lateral, old_lateral);
      return true;
    }

    const auto old_target = local_target_pt_;
    const auto old_velocity = local_target_vel_;
    auto &global_progress = planner_manager_->global_data_.last_progress_time_;
    const double old_progress = global_progress;
    bool replaced = false;
    bool stair = false;
    // Shorten the guide progressively when a stair edge cuts the FREE band.
    // A short recovery request already owns its endpoint and lateral offset.
    const std::array<double, 3> horizons = {short_horizon, 1.2, 0.9};
    const size_t count = short_horizon > 0.0 ? 1 : horizons.size();
    for (size_t attempt = 0; attempt < count; ++attempt)
    {
      const double horizon = horizons[attempt];
      if (attempt > 0)
        getLocalTarget(horizon);
      UniformBspline alternate;
      if (!solveLocalCandidate(alternate, true, false, true, horizon > 0.0, false, stair))
        continue;
      const auto clear = [&](const Eigen::Vector3d &p, double heading) {
        return planner_manager_->grid_map_->getInflateOccupancy(p, heading) == 0 &&
               bodyInsideFree(p, heading, referencePosition(progress(p)).z() - body_height_);
      };
      // The spline may start at the old execution sample instead of odometry.
      const Eigen::Vector3d gap = alternate.evaluateDeBoorT(0.0) - odom_pos_;
      const int steps = std::max(1, static_cast<int>(std::ceil(gap.norm() / 0.025)));
      bool connected = true;
      for (int i = 0; connected && i <= steps; ++i)
        connected = clear(odom_pos_ + gap * (static_cast<double>(i) / steps), yaw);
      if (!connected)
        continue;
      const auto preview = previewTurns(alternate, yaw, turn_lookahead_, turn_threshold_, turn_kp_,
                                        turn_rate_, clear);
      const double advance = progress(alternate.evaluateDeBoorT(preview.phase)) - origin;
      const double minimum_advance = incumbent.clear ? std::max(0.05, old_advance) : 0.05;
      if (!preview.clear || preview.longest_turn > turn_budget || advance < minimum_advance)
        continue;
      selected = alternate;
      replaced = true;
      short_horizon = horizon;
      break;
    }
    if (!replaced)
    {
      local_target_pt_ = old_target;
      local_target_vel_ = old_velocity;
      global_progress = old_progress;
    }
    ROS_INFO("Stair handoff preview: clear=%d turn=%.2f s selected=%s horizon=%.2f", incumbent.clear,
             incumbent.longest_turn, replaced ? "forward" : (incumbent.clear ? "original" : "none"), short_horizon);
    return replaced || incumbent.clear;
  }


  double SCANReplanFSM::raisedReplanStartHeight() const
  {
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH || reference_route_.size() < 2)
      return start_pt_.z();

    // Compare XY at the measured body, within its current vertical neighborhood.
    // A 3D nearest point would slide backward down the flight when the body crouches.
    double nearest_xy = 0.30 * 0.30;
    double reference_z = start_pt_.z();
    for (size_t i = 1; i < reference_route_.size(); ++i)
    {
      const Eigen::Vector3d from = reference_route_[i - 1];
      const Eigen::Vector3d step = reference_route_[i] - from;
      const double length_xy = step.head<2>().squaredNorm();
      if (length_xy < 1e-8)
        continue;
      const double fraction = std::clamp(
          (odom_pos_ - from).head<2>().dot(step.head<2>()) / length_xy, 0.0, 1.0);
      const Eigen::Vector3d projected = from + fraction * step;
      if (std::abs(projected.z() - odom_pos_.z()) > 0.75)
        continue;
      const double error = (projected - odom_pos_).head<2>().squaredNorm();
      if (error < nearest_xy)
      {
        nearest_xy = error;
        reference_z = projected.z();
      }
    }
    // Bound against measured Z so successive retries cannot accumulate height.
    return std::max(start_pt_.z(), std::min(reference_z, odom_pos_.z() + 0.20));
  }

  bool SCANReplanFSM::followsElevatorIngress(UniformBspline &candidate)
  {
    if (ingress_direction_.isZero()) return true;
    const double duration = candidate.getTimeSum();
    const Eigen::Vector2d start = candidate.evaluateDeBoorT(0.).head<2>();
    double lookahead = .8;
    ros::param::getCached("/closed_loop_controller/time_forward", lookahead);
    for (double t = 0.; t <= std::min(duration, 1.2); t += .10)
    {
      const Eigen::Vector2d pos = candidate.evaluateDeBoorT(t).head<2>();
      const Eigen::Vector2d heading = headingPreview(candidate, t, duration, lookahead, pos);
      if ((!heading.isZero(1e-9) && heading.normalized().dot(ingress_direction_) < std::cos(.65)) ||
          (pos - start).dot(ingress_direction_) < -.04)
        return false;
    }
    return true;
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj,
                                     double short_horizon, double lateral_offset, bool use_reference_target)
  {
    if (use_reference_target)
    {
      if (!getShortReferenceTarget(short_horizon))
        return false;
    }
    else
      getLocalTarget(short_horizon);
    if (lateral_offset != 0.0)
    {
      const double yaw = estimateYawFromSegment(start_pt_, local_target_pt_);
      local_target_pt_.x() -= std::sin(yaw) * lateral_offset;
      local_target_pt_.y() += std::cos(yaw) * lateral_offset;
      if (!bodyInsideFree(local_target_pt_, yaw, local_target_pt_.z() - body_height_) ||
          planner_manager_->grid_map_->getInflateOccupancy(local_target_pt_, yaw) != 0)
        return false;
    }
    if (short_horizon > 0.0 && (local_target_pt_ - start_pt_).head<2>().norm() < 0.20)
      return false;

    bool stair = false;
    UniformBspline selected;
    bool plan_success = solveLocalCandidate(selected, have_new_target_ || flag_use_poly_init,
                                            flag_randomPolyTraj, use_reference_target, short_horizon > 0.0,
                                            lateral_offset != 0.0, stair);
    if (!plan_success && !use_reference_target && navi_mode_ == NAVI_MODE::REFERENCE_PATH && reference_distance_.size() >= 2 &&
        std::abs(local_target_pt_.z() - start_pt_.z()) >= 0.20)
      plan_success = solveLocalCandidate(selected, true, false, true, short_horizon > 0.0,
                                         lateral_offset != 0.0, stair);
    if (plan_success && stair && !selectStairCandidate(selected, short_horizon))
      plan_success = false;
    if (!plan_success)
    {
      const double original_z = start_pt_.z();
      const double target_z = raisedReplanStartHeight();
      const double original_horizon = short_horizon;
      for (double raise = 0.05; original_z + raise <= target_z + 0.049; raise += 0.05)
      {
        start_pt_.z() = std::min(original_z + raise, target_z);
        short_horizon = original_horizon;
        if (start_pt_.z() <= original_z + 0.025 ||
            planner_manager_->grid_map_->getInflateOccupancy(start_pt_, getOdomYaw()) != 0)
          continue;
        plan_success = solveLocalCandidate(selected, true, false, true, short_horizon > 0.0,
                                            lateral_offset != 0.0, stair);
        if (plan_success && stair && !selectStairCandidate(selected, short_horizon))
          plan_success = false;
        if (plan_success)
        {
          ROS_INFO("Raised replan start: body_z=%.3f original_z=%.3f selected_z=%.3f; measured connector checked",
                   odom_pos_.z(), original_z, start_pt_.z());
          break;
        }
      }
      if (!plan_success)
      {
        start_pt_.z() = original_z;
        short_horizon = original_horizon;
      }
    }
    have_new_target_ = false;
    if (plan_success)
    {
      planner_manager_->commitCandidate(selected);
      auto info = &planner_manager_->local_data_;

      if (holding_emergency_stop_)
        ROS_INFO("Resuming with a checked local trajectory: distance=%.2f lateral=%.2f.",
                 short_horizon, lateral_offset);
      holding_emergency_stop_ = false;
      stopped_conflict_.reset();
      last_failed_full_replan_ = ros::Time();
      short_replan_time_.reset();
      if (short_horizon > 0.0)
      {
        short_replan_time_ = stair ? shortReplanTime(info->position_traj_) : 0.5 * info->duration_;
        ROS_INFO("Short trajectory %d: horizon=%.2f lateral=%.2f, request successor at %.3f of %.3f s",
                 info->traj_id_, short_horizon, lateral_offset, *short_replan_time_, info->duration_);
      }
      /* publish traj */
      scan_upstream_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    holding_emergency_stop_ = true;
    short_replan_time_.reset();
    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_upstream_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  bool SCANReplanFSM::getShortReferenceTarget(double horizon)
  {
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH || reference_distance_.size() < 2)
      return false;
    // A polynomial fitted across a tight landing can cut its corner.
    // Seed short recovery on the connected, same-height FREE reference.
    const double total = reference_distance_.back();
    const double origin = referenceProgress(start_pt_, 0.0, total);
    const auto &map = planner_manager_->grid_map_;
    for (double distance = std::min(horizon, total - origin); distance >= 0.20; distance -= 0.08)
    {
      const double progress = origin + distance;
      const Eigen::Vector3d point = referencePosition(progress);
      const double yaw = estimateYawFromSegment(referencePosition(progress - 0.12),
                                                referencePosition(progress + 0.12));
      if ((point - start_pt_).head<2>().norm() < 0.20 ||
          map->getInflateOccupancy(point, yaw) != 0 ||
          !bodyInsideFree(point, yaw, point.z() - body_height_))
        continue;
      local_target_pt_ = point;
      local_target_vel_.setZero();
      return true;
    }
    return false;
  }

  void SCANReplanFSM::getLocalTarget(double short_horizon)
  {
    const double max_vel = planner_manager_->pp_.max_vel_;
    const double max_acc = planner_manager_->pp_.max_acc_;
    const double duration = planner_manager_->global_data_.global_duration_;
    const double horizon = short_horizon > 0.0 ? short_horizon : planning_horizon_;
    double t_step = max_vel > 1e-6 ? horizon / 20.0 / max_vel : 0.01;
    t_step = std::max(t_step, 0.01);

    double t_proj = 0.0;
    double min_dist_to_start = 9999.0;
    for (double t = 0.0; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist_to_start = (pos_t - start_pt_).norm();
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        t_proj = t;
      }
    }

    double target_t = duration;
    double total_dist = 0.0;
    bool target_found = false;
    Eigen::Vector3d prev_pos = planner_manager_->global_data_.getPosition(t_proj);
    local_target_pt_ = end_pt_;

    for (double t = t_proj; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      total_dist += (pos_t - prev_pos).norm();
      if (total_dist >= horizon)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        target_found = true;
        break;
      }
      prev_pos = pos_t;
    }
    planner_manager_->global_data_.last_progress_time_ = target_found ? target_t : duration;

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;
        if ((short_horizon <= 0.0 || dt == 0.0) && t_forward <= duration)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, t_proj))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        ROS_WARN_THROTTLE(1.0, "Local target in collision, adjusted to a nearby collision-free point.");
        target_t = adjusted_t;
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "Local target in collision and no nearby collision-free target was found.");
      }
    }

    if (short_horizon > 0.0 || (end_pt_ - local_target_pt_).norm() < (max_vel * max_vel) / (2 * max_acc))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      if (local_target_vel_.norm() > max_vel)
        local_target_vel_ = local_target_vel_.normalized() * max_vel;
      // cout << "AA" << endl;
    }
  }

} // namespace scan_upstream_planner
