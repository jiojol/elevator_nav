#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Eigen>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <plan_manage/heading_preview.hpp>
#include <tf/tf.h>

#include "bspline_opt/uniform_bspline.h"
#include "scan_upstream_planner/Bspline.h"

namespace
{
using scan_upstream_planner::UniformBspline;

constexpr double kMaxVYawLimit = 1.0;

ros::Publisher cmd_vel_pub, rl_cmd_vel_pub;
ros::Time spline_stamp;
ros::Publisher execution_frozen_pub;
ros::Publisher execution_progress_pub;

bool receive_traj = false;
bool have_odom = false;
std::vector<UniformBspline> traj;
double traj_duration = 0.0;
int traj_id = 0;

Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;
bool stationary_hold = false;
Eigen::Vector2d chassis_velocity = Eigen::Vector2d::Zero();  // base_link
ros::Time chassis_stamp, pose_stamp;

double exec_time = 0.0;
ros::Time last_update_time;

double time_forward;
double heading_error_threshold;
double kp_pos;
double kp_yaw;
double max_vx;
double max_vy;
double max_vyaw;
double finish_dist;
std::string body_pose_topic;

bool loadRequiredParam(const ros::NodeHandle &nh, const std::string &name, double &value)
{
  if (nh.getParam(name, value))
    return true;

  ROS_ERROR_STREAM("[closed_loop_controller] missing required private parameter ~" << name);
  return false;
}

bool loadParams(const ros::NodeHandle &nh)
{
  bool ok = true;
  ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  ok &= loadRequiredParam(nh, "time_forward", time_forward);
  ok &= loadRequiredParam(nh, "heading_error_threshold", heading_error_threshold);
  ok &= loadRequiredParam(nh, "kp_pos", kp_pos);
  ok &= loadRequiredParam(nh, "kp_yaw", kp_yaw);
  ok &= loadRequiredParam(nh, "max_vx", max_vx);
  ok &= loadRequiredParam(nh, "max_vy", max_vy);
  ok &= loadRequiredParam(nh, "max_vyaw", max_vyaw);
  ok &= loadRequiredParam(nh, "finish_dist", finish_dist);
  if (ok && max_vyaw > kMaxVYawLimit)
  {
    ROS_WARN("[closed_loop_controller] cap max_vyaw %.3f to %.3f rad/s.", max_vyaw, kMaxVYawLimit);
    max_vyaw = kMaxVYawLimit;
  }
  return ok;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
{
  const double norm = value.norm();
  if (norm <= max_norm || norm < 1e-6)
    return value;
  return value / norm * max_norm;
}

double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des)
{
  const Eigen::Vector2d dir = scan_upstream_planner::headingPreview(
      traj[0], t_cur, traj_duration, time_forward, pos_des.head<2>());
  return dir.isZero(1e-9) ? odom_yaw : std::atan2(dir.y(), dir.x());
}

void publishCommand(const geometry_msgs::Twist &cmd)
{
  cmd_vel_pub.publish(cmd);
  if (rl_cmd_vel_pub)
  {
    geometry_msgs::TwistStamped stamped;
    stamped.header.stamp = spline_stamp;
    stamped.twist = cmd;
    rl_cmd_vel_pub.publish(stamped);
  }
}

void publishStop(double vyaw = 0.0)
{
  geometry_msgs::Twist cmd;
  cmd.angular.z = clamp(vyaw, -max_vyaw, max_vyaw);
  publishCommand(cmd);
}

void publishExecutionFrozen(bool frozen)
{
  std_msgs::Bool msg;
  msg.data = frozen;
  execution_frozen_pub.publish(msg);
}

void bsplineCallback(const scan_upstream_planner::BsplineConstPtr &msg)
{
  if (msg->pos_pts.empty())
    return;

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());

  for (size_t i = 0; i < msg->knots.size(); ++i)
    knots(i) = msg->knots[i];

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // EmergencyStop encodes a stop by publishing identical position controls.
  stationary_hold = (pos_pts.colwise() - pos_pts.col(0).eval()).cwiseAbs().maxCoeff() <= 1e-9;
  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj_duration = traj[0].getTimeSum();
  traj_id = msg->traj_id;
  spline_stamp = msg->start_time;
  exec_time = 0.0;
  last_update_time = ros::Time::now();
  receive_traj = true;

  ROS_WARN("[closed_loop_controller] received bspline traj_id=%d duration=%.3f", traj_id, traj_duration);
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;
  odom_yaw = tf::getYaw(msg->pose.pose.orientation);
  pose_stamp = msg->header.stamp;
  have_odom = true;
}

void chassisVelocityCallback(const nav_msgs::OdometryConstPtr &msg)
{
  const auto &v = msg->twist.twist.linear;
  const auto stamp = msg->header.stamp;
  if (msg->child_frame_id != "base_link" || !std::isfinite(v.x) || !std::isfinite(v.y) ||
      stamp.isZero() || stamp > ros::Time::now() || stamp <= chassis_stamp)
    return;

  const Eigen::Vector2d sample(v.x, v.y);
  const double dt = (stamp - chassis_stamp).toSec();
  if (chassis_stamp.isZero() || dt > 0.20)
    chassis_velocity = sample;
  else
    chassis_velocity += -std::expm1(-dt / 0.12) * (sample - chassis_velocity);
  chassis_stamp = stamp;
}

double stairSpeedCorrection(const Eigen::Vector3d &position, const Eigen::Vector2d &feedforward,
                            const Eigen::Vector2d &position_error, const Eigen::Vector2d &command,
                            const ros::Time &now)
{
  const double age = (now - chassis_stamp).toSec();
  const double pose_age = (now - pose_stamp).toSec();
  const double speed = feedforward.norm();
  if (chassis_stamp.isZero() || age < 0.0 || age > 0.20 || pose_age < 0.0 || pose_age > 0.20 ||
      speed < 0.03 || exec_time >= traj_duration)
    return 0.0;

  const Eigen::Vector2d tangent = feedforward / speed;
  const double lag = tangent.dot(position_error);
  const Eigen::Vector2d body_forward(std::cos(odom_yaw), std::sin(odom_yaw));
  if (lag <= 0.0 || tangent.dot(body_forward) < 0.90 ||
      tangent.dot(command) < 0.90 * command.norm())
    return 0.0;

  const Eigen::Vector3d ahead = traj[0].evaluateDeBoorT(std::min(exec_time + 2.0, traj_duration));
  const Eigen::Vector2d forward = (ahead - position).head<2>();
  if (tangent.dot(forward) < std::max(0.10, 0.90 * forward.norm()))
    return 0.0;

  const Eigen::Vector3d behind = traj[0].evaluateDeBoorT(std::max(0.0, exec_time - 0.6));
  const Eigen::Vector3d span = ahead - behind;
  const double grade = std::abs(span.z()) / std::max(0.10, span.head<2>().norm());
  const double grade_weight = std::clamp((grade - 0.12) / 0.15, 0.0, 1.0);
  const Eigen::Vector2d body_left(-body_forward.y(), body_forward.x());
  const double measured = tangent.dot(body_forward) * chassis_velocity.x() +
                          tangent.dot(body_left) * chassis_velocity.y();

  // Correct only a positive shortfall; position lag and reference speed
  // both bound the extra command as the chassis catches up or the curve ends.
  return grade_weight * std::max(0.0, std::min({0.10, 1.2 * lag, 0.70 * (speed - measured), 0.50 * speed}));
}

void publishExecutionProgress()
{
  std_msgs::Float64MultiArray msg;
  msg.data = {static_cast<double>(traj_id), exec_time};
  execution_progress_pub.publish(msg);
}

void cmdCallback(const ros::TimerEvent &)
{
  if (!receive_traj || !have_odom || stationary_hold)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  const ros::Time now = ros::Time::now();
  double dt = (now - last_update_time).toSec();
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  const double t_eval = std::min(exec_time, traj_duration);
  Eigen::Vector3d pos_des = traj[0].evaluateDeBoorT(t_eval);
  Eigen::Vector3d vel_des = traj[1].evaluateDeBoorT(t_eval);

  const double yaw_des = estimateDesiredYaw(t_eval, pos_des);
  const double yaw_err = normalizeAngle(yaw_des - odom_yaw);
  const double vyaw_cmd = clamp(kp_yaw * yaw_err, -max_vyaw, max_vyaw);

  if (std::abs(yaw_err) > heading_error_threshold)
  {
    publishExecutionFrozen(true);
    publishStop(vyaw_cmd);
    publishExecutionProgress();
    last_update_time = now; // freeze exec_time while rotating in place
    return;
  }

  // Keep commanding progress toward the current sample when the chassis
  // lags. Advancing time in that case would inspect a different stair step.
  constexpr double max_tracking_lead = 0.35;
  const Eigen::Vector2d error = pos_des.head<2>() - odom_pos.head<2>();
  const Eigen::Vector2d direction(std::cos(yaw_des), std::sin(yaw_des));
  const bool waiting_for_position = error.dot(direction) > max_tracking_lead;
  publishExecutionFrozen(waiting_for_position);
  if (!waiting_for_position)
    exec_time = std::min(traj_duration, exec_time + dt);
  publishExecutionProgress();
  last_update_time = now;

  pos_des = traj[0].evaluateDeBoorT(exec_time);
  vel_des = traj[1].evaluateDeBoorT(exec_time);

  const Eigen::Vector2d pos_err = pos_des.head<2>() - odom_pos.head<2>();
  const Eigen::Vector2d vel_ff = vel_des.head<2>();
  Eigen::Vector2d position_feedback = pos_err;
  if (exec_time < traj_duration && vel_ff.norm() > 0.18 && std::abs(yaw_err) < 0.25 &&
      direction.dot(vel_ff) >= 0.97 * vel_ff.norm())
  {
    // Suppress centimetre-scale corrections on straight stretches. Preserve
    // the signed velocity feedforward along a checked avoidance trajectory.
    const Eigen::Vector2d normal(-direction.y(), direction.x());
    position_feedback -= clamp(normal.dot(pos_err), -0.035, 0.035) * normal;
  }
  const Eigen::Vector2d vel_world = clampNorm(vel_ff + kp_pos * position_feedback, std::max(max_vx, max_vy));

  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::Twist cmd;
  cmd.linear.x = clamp(c * vel_world(0) + s * vel_world(1), -max_vx, max_vx);
  cmd.linear.y = clamp(-s * vel_world(0) + c * vel_world(1), -max_vy, max_vy);
  cmd.angular.z = vyaw_cmd;

  const double correction = stairSpeedCorrection(pos_des, vel_ff, pos_err, vel_world, now);
  const double command_speed = std::hypot(cmd.linear.x, cmd.linear.y);
  if (correction > 0.0 && command_speed > 1e-4)
  {
    // Scale the already bounded body command so a saturated lateral axis
    // cannot change the tracking direction.
    double scale = std::min(command_speed + correction, std::max(max_vx, max_vy)) / command_speed;
    if (cmd.linear.x != 0.0)
      scale = std::min(scale, max_vx / std::abs(cmd.linear.x));
    if (cmd.linear.y != 0.0)
      scale = std::min(scale, max_vy / std::abs(cmd.linear.y));
    cmd.linear.x *= scale;
    cmd.linear.y *= scale;
  }

  if (exec_time >= traj_duration)
  {
    const Eigen::Vector2d approach = pos_des.head<2>() -
        traj[0].evaluateDeBoorT(std::max(0.0, traj_duration - 0.5)).head<2>();
    // The next segment should start from odometry. Once past the endpoint,
    // do not pull the chassis back to the end of an exhausted short segment.
    if (pos_err.norm() < finish_dist ||
        (approach.squaredNorm() > 1e-6 && pos_err.dot(approach) < 0.0))
      cmd = geometry_msgs::Twist();
  }

  publishCommand(cmd);
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "closed_loop_controller");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  if (!loadParams(nh))
    return 1;

  const auto bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  const auto odom_sub = node.subscribe(body_pose_topic, 20, odomCallback, ros::TransportHints().tcpNoDelay());
  const auto chassis_velocity_sub = node.subscribe("/go2/odom", 5, chassisVelocityCallback, ros::TransportHints().tcpNoDelay());
  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 20);
  bool rl_control = false;
  ros::param::param("/integration/go2_rl", rl_control, false);
  if (rl_control)
    rl_cmd_vel_pub = node.advertise<geometry_msgs::TwistStamped>("/planning/scan_upstream_stamped_cmd_vel", 1);
  execution_frozen_pub = node.advertise<std_msgs::Bool>("planning/go2_execution_frozen", 10);
  execution_progress_pub = node.advertise<std_msgs::Float64MultiArray>("planning/go2_execution_progress", 1);
  auto cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  last_update_time = ros::Time::now();
  ROS_WARN("[closed_loop_controller] ready.");

  ros::spin();
  // Release the ROS handles while their managers are still alive.
  cmd_timer.stop();
  execution_progress_pub.shutdown();
  execution_frozen_pub.shutdown();
  rl_cmd_vel_pub.shutdown();
  cmd_vel_pub.shutdown();
  return 0;
}
