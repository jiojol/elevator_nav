#pragma once
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <scan_planner/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <plan_manage/collision_progress.hpp>

namespace scan_planner {
// One owner of route acceptance, native collision replanning and emergency
// state. The hardware adapter may HOLD/ACK, but never chooses another route.
class SCANReplanFSM {
 public:
  void init(ros::NodeHandle& nh);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  enum State { WAIT_TARGET, GENERATE, EXECUTE, REPLAN, EMERGENCY, ALIGN_START };
  State state_=WAIT_TARGET;
  SCANPlannerManager::Ptr manager_;
  PlanningVisualization::Ptr visualization_;
  ReferencePath route_;
  Eigen::Vector3d position_=Eigen::Vector3d::Zero(),velocity_=Eigen::Vector3d::Zero();
  Eigen::Vector3d start_=Eigen::Vector3d::Zero(),start_velocity_=Eigen::Vector3d::Zero(),start_acceleration_=Eigen::Vector3d::Zero();
  Eigen::Vector3d target_=Eigen::Vector3d::Zero(),target_velocity_=Eigen::Vector3d::Zero(),last_projection_pose_=Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_=Eigen::Quaterniond::Identity();
  bool have_odom_=false,have_target_=false,frozen_=false,stop_latched_=false,waiting_=false,projected_=false,twist_in_body_=false;
  double body_height_=.4,horizon_=3.5,replan_distance_=1.,no_replan_distance_=.1,emergency_time_=1.,progress_=0.;
  bool fail_safe_=true;
  int failures_=0,navi_mode_=3;
  bool alignment_active_=false;
  unsigned long alignment_sequence_=0;
  double alignment_start_yaw_=0.,alignment_target_=0.,alignment_angle_=0.,alignment_padding_=.2;
  Eigen::Vector3d alignment_anchor_=Eigen::Vector3d::Zero();
  ros::Time alignment_last_publish_,retry_started_,retry_cloud_stamp_;
  double last_solve_ms_=0.;
  CollisionProgress collision_progress_;
  double collision_distance_=-1.,braking_distance_=0.;
  ros::Time odom_stamp_,odom_received_,route_stamp_,cancel_stamp_,freeze_updated_,retry_at_;
  std::string frame_="map";
  Eigen::Vector2d reference_offset_=Eigen::Vector2d::Zero();
  ros::Subscriber odom_sub_,route_sub_,goal_sub_,frozen_sub_,handoff_sub_,hold_sub_,position_reached_sub_,cancel_sub_,alignment_done_sub_;
  ros::Publisher spline_pub_,collision_pub_,status_pub_,tracking_path_pub_,alignment_pub_;
  ros::Timer exec_timer_,collision_timer_;
  void transition(State next,const char* reason);
  void status(const std::string& reason);
  void odometry(const nav_msgs::OdometryConstPtr& msg);
  void path(const nav_msgs::PathConstPtr& msg);
  void goal(const geometry_msgs::PoseStampedConstPtr& msg);
  void frozen(const std_msgs::BoolConstPtr& msg);
  void handoff(const std_msgs::Int64ConstPtr& msg);
  void hold(const std_msgs::StringConstPtr& msg);
  void arrived(const std_msgs::HeaderConstPtr& msg);
  void cancel(const std_msgs::EmptyConstPtr& msg);
  void execute(const ros::TimerEvent&);
  void collision(const ros::TimerEvent&);
  void updateClock();
  bool freshOdom() const;
  double yaw() const;
  bool selectTarget();
  bool plan(bool from_trajectory,bool random=false);
  bool replan();
  bool beginStartAlignment();
  void updateStartAlignment();
  void publishStartAlignment(bool allowed,bool cancel=false);
  void alignmentDone(const std_msgs::StringConstPtr& msg);
  void publishTrajectory(bool obstacle_hold=false);
  void stop(const std::string& reason,bool notify=true);
};
}
