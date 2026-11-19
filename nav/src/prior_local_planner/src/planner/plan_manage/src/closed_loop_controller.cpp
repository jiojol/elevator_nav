#include <std_msgs/Header.h>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <tf/tf.h>

#include <bspline_opt/uniform_bspline.h>
#include <plan_manage/go2_tracking_policy.hpp>
#include <plan_manage/reference_path.hpp>
#include <scan_planner/Bspline.h>

namespace
{
using scan_planner::UniformBspline;
using scan_planner::Go2TrackingPolicy;
using scan_planner::ReferencePath;

ros::Publisher cmd_vel_pub, execution_frozen_pub, status_pub, segment_end_pub, execution_stall_pub;
ros::Subscriber bspline_sub, odom_sub, reference_sub, position_reached_sub;
ros::Timer cmd_timer;
bool terrain_tracking=false;
bool receive_traj = false, have_odom = false, have_previous_pose = false;
bool reference_known = false, reference_active = false;
std::vector<UniformBspline> traj;
ReferencePath curve_path;
std::vector<double> curve_arc, curve_time;
Go2TrackingPolicy tracker;
double traj_duration = 0.0, exec_time = 0.0, body_progress = 0.0;
int traj_id = 0;
Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero(), previous_pose = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;
ros::Time last_update_time, odom_stamp, odom_received, reference_stamp, curve_stamp, last_status;
double time_forward, heading_error_threshold, kp_pos, kp_yaw, max_vx, max_vy, max_vyaw, finish_dist;
double lookahead_distance = 0.35, trajectory_lead = 0.20, odom_timeout = 0.50;
double segment_handoff_time = 1.0;
double no_progress_timeout=3.0,no_progress_distance=.05,no_progress_yaw=.05,no_progress_command=.03;
bool progress_probe_active=false;
ros::SteadyTime progress_probe_since;
double progress_probe_arc=0.,progress_probe_target_heading=0.,progress_probe_heading_error=0.;
double body_height = 0.40, scheduled_speed = 0.0, terminal_speed = 0.0, remaining_arc = 0.0;
Eigen::Vector3d reference_goal = Eigen::Vector3d::Zero();
bool handoff_requested = false, align_on_resume = false;
std::string body_pose_topic;

bool required(const ros::NodeHandle &nh, const std::string &name, double &value)
{
  if (nh.getParam(name, value)) return true;
  ROS_ERROR_STREAM("[closed_loop_controller] missing required private parameter ~" << name);
  return false;
}

bool loadParams(const ros::NodeHandle &nh)
{
  ros::param::param<std::string>("/body_pose_topic", body_pose_topic, "/quad_0/body_pose");
  bool ok = true;
  ok &= required(nh, "time_forward", time_forward);
  ok &= required(nh, "heading_error_threshold", heading_error_threshold);
  ok &= required(nh, "kp_pos", kp_pos);
  ok &= required(nh, "kp_yaw", kp_yaw);
  ok &= required(nh, "max_vx", max_vx);
  ok &= required(nh, "max_vy", max_vy);
  ok &= required(nh, "max_vyaw", max_vyaw);
  ok &= required(nh, "finish_dist", finish_dist);
  if (!ok) return false;
  auto &limits = tracker.limits;
  limits.max_forward = max_vx;
  limits.max_yaw = std::min(1.0, max_vyaw);
  limits.rotate_enter = heading_error_threshold;
  limits.position_gain = kp_pos;
  limits.yaw_gain = kp_yaw;
  limits.finish_distance = finish_dist;
  nh.param("rotate_exit_threshold", limits.rotate_exit, 0.25);
  nh.param("start_align_enter", limits.start_align_enter, 0.25);
  nh.param("start_align_exit", limits.start_align_exit, 0.10);
  nh.param("start_align_min_yaw", limits.start_align_min_yaw, 0.40);
  nh.param("linear_accel", limits.linear_accel, 0.25);
  nh.param("angular_accel", limits.angular_accel, 0.80);
  nh.param("heading_filter_time", limits.heading_filter_time, 0.20);
  nh.param("min_forward_speed", limits.min_forward, 0.04);
  nh.param("lookahead_distance", lookahead_distance, 0.35);
  nh.param("trajectory_lead", trajectory_lead, 0.20);
  nh.param("segment_handoff_time", segment_handoff_time, 1.0);
  nh.param("no_progress_timeout",no_progress_timeout,3.0);
  nh.param("no_progress_distance",no_progress_distance,.05);
  nh.param("no_progress_yaw",no_progress_yaw,.05);
  nh.param("no_progress_command",no_progress_command,.03);
  nh.param("odom_timeout", odom_timeout, 0.50);
  nh.param("terrain_tracking",terrain_tracking,false);
  ros::param::param("/scan_planner_node/grid_map/body_height", body_height, 0.40);
  const double positive[] = {time_forward, max_vx, max_vyaw, finish_dist, kp_pos, kp_yaw,
      limits.rotate_enter, limits.rotate_exit, limits.start_align_enter, limits.start_align_exit, limits.linear_accel, limits.angular_accel,
      lookahead_distance, trajectory_lead, odom_timeout, segment_handoff_time,
      no_progress_timeout,no_progress_distance,no_progress_yaw,no_progress_command};
  for (const double value : positive) ok &= std::isfinite(value) && value > 0.0;
  ok &= std::isfinite(limits.heading_filter_time) && limits.heading_filter_time >= 0.0;
  ok &= std::isfinite(limits.min_forward) && limits.min_forward >= 0.0 && limits.min_forward <= max_vx;
  ok &= limits.rotate_exit < limits.rotate_enter && limits.rotate_enter < 1.5708;
  ok &= limits.start_align_exit < limits.start_align_enter && limits.start_align_enter <= limits.rotate_enter;
  ok &= std::isfinite(limits.start_align_min_yaw) && limits.start_align_min_yaw>=0.;
  if (!ok) ROS_ERROR("Invalid Go2 forward tracking parameters.");
  return ok;
}

void publishFrozen(bool frozen)
{
  std_msgs::Bool msg;
  msg.data = frozen;
  execution_frozen_pub.publish(msg);
}

void stop()
{
  progress_probe_active=false;
  tracker.reset();
  cmd_vel_pub.publish(geometry_msgs::Twist());
}

void status(const std::string &mode, bool frozen, double heading_error = 0.0)
{
  const ros::Time now = ros::Time::now();
  if (!last_status.isZero() && (now - last_status).toSec() < 0.20) return;
  last_status = now;
  std::ostringstream text;
  text << "{\"mode\":\"" << mode << "\",\"trajectory_id\":" << traj_id
       << ",\"execution_time\":" << exec_time << ",\"body_progress\":" << body_progress
       << ",\"heading_error\":" << heading_error << ",\"frozen\":" << (frozen ? "true" : "false")
       << ",\"forward_only\":true,\"scheduled_speed\":" << scheduled_speed
       << ",\"requested_speed\":" << tracker.command().requested_forward
       << ",\"turn_speed_limit\":" << tracker.command().turn_limit
       << ",\"brake_speed_limit\":" << tracker.command().brake_limit
       << ",\"command_forward\":" << tracker.command().forward
       << ",\"command_yaw_rate\":" << tracker.command().yaw_rate
       << ",\"remaining_arc\":" << remaining_arc << ",\"terminal_speed\":" << terminal_speed
       << ",\"handoff_requested\":" << (handoff_requested ? "true" : "false") << "}";
  std_msgs::String msg;
  msg.data = text.str();
  status_pub.publish(msg);
}

double interpolate(const std::vector<double> &axis, const std::vector<double> &values, double value)
{
  if (value <= axis.front()) return values.front();
  const auto upper = std::upper_bound(axis.begin(), axis.end(), value);
  if (upper == axis.end()) return values.back();
  const size_t i = static_cast<size_t>(upper - axis.begin());
  const double fraction = (value - axis[i - 1]) / std::max(1e-12, axis[i] - axis[i - 1]);
  return values[i - 1] + fraction * (values[i] - values[i - 1]);
}

void referenceCallback(const nav_msgs::PathConstPtr &msg)
{
  if (!msg->poses.empty() && !reference_stamp.isZero() && msg->header.stamp <= reference_stamp) return;
  reference_known = true;
  align_on_resume = false;
  reference_active = !msg->poses.empty();
  if (reference_active)
  {
    const auto &p = msg->poses.back().pose.position;
    reference_goal << p.x, p.y, p.z + body_height;
  }
  reference_stamp = reference_active ? msg->header.stamp : ros::Time::now();
  if (!reference_active || curve_stamp < reference_stamp) receive_traj = false;
  // A new goal stops the old command. A rolling/collision spline handoff does
  // not reset the velocity ramp or heading filter.
  stop();
  last_update_time = ros::Time::now();
}

void positionReachedCallback(const std_msgs::HeaderConstPtr& msg)
{
  if (!reference_active || msg->stamp!=reference_stamp) return;
  // The hardware adapter now owns final yaw. Stop the old curve's tracker,
  // clock and no-progress probe instead of letting them fight its intent.
  receive_traj=false;reference_active=false;align_on_resume=false;
  publishFrozen(false);stop();status("final_orientation",false);
}

void bsplineCallback(const scan_planner::BsplineConstPtr &msg)
{
  if ((reference_known && !reference_active) || msg->start_time < reference_stamp) return;
  const size_t count = msg->pos_pts.size();
  bool valid = msg->order == 3 && count >= 4 && count <= 2000 && msg->knots.size() == count + 4;
  for (size_t i = 0; i < msg->knots.size(); ++i)
    valid &= std::isfinite(msg->knots[i]) && (i == 0 || msg->knots[i] > msg->knots[i - 1]);
  for (const auto &point : msg->pos_pts)
    valid &= std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
  if (!valid)
  {
    receive_traj = false; stop();
    ROS_ERROR("Rejected malformed B-spline before tracking.");
    return;
  }
  Eigen::MatrixXd points(3, count);
  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < count; ++i) points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
  for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
  UniformBspline position(points, 3, 0.1);
  position.setKnot(knots);
  progress_probe_active=false;
  traj_duration = position.getTimeSum();
  if (!std::isfinite(traj_duration) || traj_duration <= 0.0 || traj_duration > 120.0)
  { receive_traj = false; stop(); return; }
  if (msg->obstacle_hold) align_on_resume=true;
  traj.clear();
  traj.push_back(position);
  traj.push_back(position.getDerivative());
  curve_arc.clear(); curve_time.clear();
  std::vector<Eigen::Vector3d> samples;
  const int sample_count = std::max(2, static_cast<int>(std::ceil(traj_duration / 0.05)) + 1);
  for (int i = 0; i < sample_count; ++i)
  {
    const double t = traj_duration * i / (sample_count - 1);
    samples.push_back(position.evaluateDeBoorT(t));
    curve_time.push_back(t);
    curve_arc.push_back(i == 0 ? 0.0 : curve_arc.back() + (samples[i] - samples[i - 1]).norm());
  }
  traj_id = msg->traj_id;
  curve_stamp = msg->start_time;
  exec_time = body_progress = 0.0;
  handoff_requested = false;
  terminal_speed = traj[1].evaluateDeBoorT(traj_duration).head<2>().norm();
  have_previous_pose = false;
  last_update_time = ros::Time::now();
  receive_traj = curve_path.set(samples) && curve_arc.back() > 1e-6;
  if (!receive_traj) stop();
  else if (align_on_resume) {
    tracker.alignBeforeNextMotion();
    align_on_resume=false;
  }
  ROS_WARN("[closed_loop_controller] received bspline traj_id=%d duration=%.3f", traj_id, traj_duration);
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  const auto &point = msg->pose.pose.position;
  const auto &orientation = msg->pose.pose.orientation;
  const double norm = orientation.x * orientation.x + orientation.y * orientation.y
                      + orientation.z * orientation.z + orientation.w * orientation.w;
  have_odom = std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)
              && std::isfinite(norm) && norm > 1e-12;
  if (!have_odom) return;
  odom_pos << point.x, point.y, point.z;
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(orientation, quaternion);
  quaternion.normalize();
  odom_yaw = tf::getYaw(quaternion);
  odom_stamp = msg->header.stamp;
  odom_received = ros::Time::now();
}

void cmdCallback(const ros::TimerEvent &)
{
  const ros::Time now = ros::Time::now();
  const double dt = (now - last_update_time).toSec();
  last_update_time = now;
  if (!receive_traj)
  { publishFrozen(false); stop(); status("waiting_for_trajectory", false); return; }
  const double age = (now - odom_stamp).toSec();
  if (!have_odom || age < -0.1 || age > odom_timeout || (now - odom_received).toSec() > odom_timeout)
  { publishFrozen(true); stop(); status("stale_odometry", true); return; }
  if (dt <= 0.0 || dt > 0.20)
  { publishFrozen(true); stop(); status("control_timer_gap", true); return; }

  const double traveled = have_previous_pose ? (odom_pos - previous_pose).norm() : 0.0;
  const auto projected = curve_path.project(odom_pos, std::max(0.0, body_progress - 0.10),
                                             body_progress + traveled + 0.35,terrain_tracking);
  body_progress = std::max(body_progress, projected.arc);
  previous_pose = odom_pos;
  have_previous_pose = true;
  const Eigen::Vector3d desired = traj[0].evaluateDeBoorT(exec_time);
  const Eigen::Vector3d velocity = traj[1].evaluateDeBoorT(exec_time);
  scheduled_speed = velocity.head<2>().norm();
  remaining_arc = std::max(0.0, curve_path.length() - body_progress);
  const Eigen::Vector3d endpoint = curve_path.at(curve_path.length());
  const Eigen::Vector3d global_delta = endpoint - reference_goal;
  const bool rolling_segment = reference_active
      && (global_delta.head<2>().norm() > 0.20 || std::abs(global_delta.z()) > 0.20);
  // Reserve planning distance using the configured speed ceiling. A slow
  // tail must not postpone its own successor request until the robot stalls.
  // Cap this at 45% of the segment to require real travel before another
  // handoff, including when an obstacle forces a shorter local segment.
  const double handoff_distance = std::min(0.45*curve_path.length(),
      std::max(0.40,std::min(1.50,max_vx*segment_handoff_time+2.0*trajectory_lead)));
  if (rolling_segment && !handoff_requested && remaining_arc <= handoff_distance
      && (endpoint - odom_pos).head<2>().norm() <= handoff_distance + 0.10)
  {
    std_msgs::Int64 complete;
    complete.data = traj_id;
    segment_end_pub.publish(complete);
    handoff_requested = true;
  }
  if (rolling_segment && handoff_requested && remaining_arc < 0.02)
  { publishFrozen(false); stop(); status("waiting_for_segment", false); return; }
  const double lookahead = std::max(lookahead_distance, std::min(max_vx, velocity.head<2>().norm()) * time_forward);
  const Eigen::Vector3d target = curve_path.at(std::min(curve_path.length(), body_progress + lookahead));
  const Eigen::Vector2d delta = (target - odom_pos).head<2>();
  const Eigen::Vector2d tangent = curve_path.tangent(body_progress).head<2>();
  const double heading = delta.norm() > 0.02 ? std::atan2(delta.y(), delta.x())
                                           : std::atan2(tangent.y(), tangent.x());
  const double along_error = (desired - odom_pos).head<2>().dot(tangent);
  const double remaining = std::max(0.0, curve_path.length() - body_progress);
  const auto command = tracker.step(heading, delta, velocity.head<2>().norm(), along_error,
                                     remaining, odom_yaw, dt, terminal_speed);
  const double next_time = std::min(traj_duration, exec_time + dt);
  const double next_arc = interpolate(curve_time, curve_arc, next_time);
  const bool frozen = command.rotating || next_arc > body_progress + trajectory_lead;
  if (!frozen) exec_time = next_time;
  publishFrozen(frozen);
  if (exec_time >= traj_duration - 1e-6 && (curve_path.at(curve_path.length()) - odom_pos).head<2>().norm() < finish_dist)
  { publishFrozen(false); stop(); status("segment_finished", false); return; }
  const bool expected_motion=(command.rotating || remaining>finish_dist+.10)
      && (command.forward>=no_progress_command || std::abs(command.yaw_rate)>.15);
  const auto steady_now=ros::SteadyTime::now();
  if (!expected_motion) progress_probe_active=false;
  else {
    // A slow curved walk can make useful heading progress before translating.
    // Compare measured yaw against a FIXED target for this probe, so moving
    // lookahead targets, oscillations and wrong-way rotation cannot renew it.
    const double probe_error=Go2TrackingPolicy::wrap(progress_probe_target_heading-odom_yaw);
    const bool heading_progress=std::abs(command.yaw_rate)>.15
        && command.yaw_rate*probe_error>=0.
        && progress_probe_heading_error-std::abs(probe_error)>=no_progress_yaw;
    const bool progressed=progress_probe_active
        && (body_progress-progress_probe_arc>=no_progress_distance || heading_progress);
    if (!progress_probe_active || progressed) {
      progress_probe_active=true;
      progress_probe_since=steady_now;progress_probe_arc=body_progress;
      progress_probe_target_heading=heading;
      progress_probe_heading_error=std::abs(Go2TrackingPolicy::wrap(heading-odom_yaw));
    } else if ((steady_now-progress_probe_since).toSec()>=no_progress_timeout) {
      const double stalled_for=(steady_now-progress_probe_since).toSec();
      std_msgs::String event;
      std::ostringstream message;
      message.precision(16);
      message << "{\"stamp\":" << now.toSec() << ",\"trajectory_id\":" << traj_id
              << ",\"reason\":\"" << (command.rotating?"turn_no_progress":"translation_no_progress")
              << "\",\"elapsed\":" << stalled_for << ",\"command_forward\":" << command.forward
              << ",\"body_progress\":" << body_progress
              << ",\"heading_error\":" << command.heading_error
              << ",\"command_yaw_rate\":" << command.yaw_rate << "}";
      event.data=message.str();
      receive_traj=false;publishFrozen(false);stop();
      execution_stall_pub.publish(event);
      status("stopped_for_no_progress",false);
      ROS_WARN("Go2 measured motion made no progress for %.2f s; stop and request a fresh local trajectory.",stalled_for);
      return;
    }
  }
  geometry_msgs::Twist output;
  output.linear.x = command.forward;
  output.linear.y = 0.0;
  output.angular.z = command.yaw_rate;
  cmd_vel_pub.publish(output);
  status(command.rotating ? "turning" : frozen ? "tracking_with_clock_held" : "tracking", frozen, command.heading_error);
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "closed_loop_controller");
  ros::NodeHandle node, nh("~");
  if (!loadParams(nh)) return 1;
  bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  odom_sub = node.subscribe(body_pose_topic, 20, odomCallback, ros::TransportHints().tcpNoDelay());
  reference_sub = node.subscribe("/planning/tracking_path", 1, referenceCallback);
  position_reached_sub=node.subscribe("/planning/goal_position_reached",10,positionReachedCallback);
  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 20);
  execution_frozen_pub = node.advertise<std_msgs::Bool>("planning/go2_execution_frozen", 10);
  status_pub = node.advertise<std_msgs::String>("planning/tracking_status", 1, true);
  segment_end_pub = node.advertise<std_msgs::Int64>("planning/local_segment_end", 10);
  execution_stall_pub=node.advertise<std_msgs::String>("/planning/execution_stall",5,true);
  last_update_time = ros::Time::now();
  cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ROS_WARN("[closed_loop_controller] ready.");
  ROS_INFO("Go2 forward-only tracking: vx<=%.2f, vy=0, yaw<=%.2f; heading hysteresis %.2f/%.2f rad.",
           max_vx, tracker.limits.max_yaw, tracker.limits.rotate_enter, tracker.limits.rotate_exit);
  ros::spin();
  return 0;
}
