// Adapt Elevator-LIO's ROS topics to the existing navigation localization contract.
//
// Elevator-LIO publishes:
//   /LIO/odom_imu                 nav_msgs/Odometry, world -> IMU body
//   /LIO/clouds_lidar/effect      sensor_msgs/PointCloud2 in the LiDAR frame
//   /LIO/in_elevator              std_msgs/Bool
//
// This adapter emits a synchronized pair with stable frame names expected by the
// map localizer:
//   /elevator_lio/odometry        camera_init -> body (IMU body pose)
//   /elevator_lio/cloud_body      PointCloud2 in body/IMU coordinates
//   /elevator_lio/in_elevator     elevator mode flag

#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace dog_nav {

using Odom = nav_msgs::Odometry;
using CloudMsg = sensor_msgs::PointCloud2;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<Odom, CloudMsg>;

class ElevatorLioAdapter {
 public:
  ElevatorLioAdapter() : pnh_("~") {
    std::string odom_topic;
    std::string cloud_topic;
    std::string elevator_flag_topic;
    std::string output_odom_topic;
    std::string output_cloud_topic;
    std::string output_elevator_flag_topic;
    pnh_.param("odom_topic", odom_topic, std::string("/LIO/odom_imu"));
    pnh_.param("cloud_topic", cloud_topic, std::string("/LIO/clouds_lidar/effect"));
    pnh_.param("elevator_flag_topic", elevator_flag_topic, std::string("/LIO/in_elevator"));
    pnh_.param("output_odom_topic", output_odom_topic, std::string("/elevator_lio/odometry"));
    pnh_.param("output_cloud_topic", output_cloud_topic, std::string("/elevator_lio/cloud_body"));
    pnh_.param("output_elevator_flag_topic", output_elevator_flag_topic,
               std::string("/elevator_lio/in_elevator"));
    pnh_.param("output_odom_frame", output_odom_frame_, std::string("camera_init"));
    pnh_.param("output_body_frame", output_body_frame_, std::string("body"));

    std::vector<double> rotation;
    std::vector<double> translation;
    if (!pnh_.getParam("imu_lidar_rotation", rotation) || rotation.size() != 9) {
      throw std::runtime_error("elevator adapter requires imu_lidar_rotation[9]");
    }
    if (!pnh_.getParam("imu_lidar_translation", translation) || translation.size() != 3) {
      throw std::runtime_error("elevator adapter requires imu_lidar_translation[3]");
    }
    imu_t_lidar_.setIdentity();
    for (int row = 0; row < 3; ++row) {
      imu_t_lidar_.translation()[row] = translation[row];
      for (int col = 0; col < 3; ++col) {
        imu_t_lidar_.linear()(row, col) = rotation[3 * row + col];
      }
    }
    if (!imu_t_lidar_.matrix().allFinite() ||
        (imu_t_lidar_.linear().transpose() * imu_t_lidar_.linear() -
         Eigen::Matrix3d::Identity()).norm() > 1.0e-3) {
      throw std::runtime_error("invalid imu_T_lidar rotation");
    }

    pnh_.param("sync_queue_size", sync_queue_size_, 30);
    pnh_.param("elevator_flag_latched", elevator_flag_latched_, true);
    if (sync_queue_size_ < 2) throw std::runtime_error("sync_queue_size must be >= 2");

    odom_pub_ = nh_.advertise<Odom>(output_odom_topic, 20);
    cloud_pub_ = nh_.advertise<CloudMsg>(output_cloud_topic, 5);
    elevator_flag_pub_ = nh_.advertise<std_msgs::Bool>(
        output_elevator_flag_topic, 1, elevator_flag_latched_);

    odom_sub_.subscribe(nh_, odom_topic, 30);
    cloud_sub_.subscribe(nh_, cloud_topic, 30);
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(sync_queue_size_), odom_sub_, cloud_sub_));
    sync_->registerCallback(boost::bind(
        &ElevatorLioAdapter::onOdomCloud, this,
        boost::placeholders::_1, boost::placeholders::_2));
    elevator_flag_sub_ = nh_.subscribe(
        elevator_flag_topic, 1, &ElevatorLioAdapter::onElevatorFlag, this);

    ROS_INFO("Elevator-LIO adapter: %s + %s -> %s + %s",
             odom_topic.c_str(), cloud_topic.c_str(), output_odom_topic.c_str(),
             output_cloud_topic.c_str());
    ROS_INFO("Elevator-LIO adapter: imu_T_lidar translation [%.6f, %.6f, %.6f]",
             imu_t_lidar_.translation().x(), imu_t_lidar_.translation().y(),
             imu_t_lidar_.translation().z());
  }

 private:
  void onOdomCloud(const Odom::ConstPtr& odom, const CloudMsg::ConstPtr& cloud) {
    if (!std::isfinite(odom->pose.pose.position.x) ||
        !std::isfinite(odom->pose.pose.position.y) ||
        !std::isfinite(odom->pose.pose.position.z)) {
      ROS_WARN_THROTTLE(2.0, "Elevator-LIO adapter rejected non-finite odometry");
      return;
    }

    Odom output_odom = *odom;
    output_odom.header.frame_id = output_odom_frame_;
    output_odom.child_frame_id = output_body_frame_;
    // The synchronized output pair must carry exactly the same stamp so the map
    // localizer's exact-time synchronizer can consume it without a second slop.
    output_odom.header.stamp = odom->header.stamp;

    pcl::PointCloud<pcl::PointXYZI> lidar_cloud;
    pcl::PointCloud<pcl::PointXYZI> body_cloud;
    pcl::fromROSMsg(*cloud, lidar_cloud);
    if (lidar_cloud.empty()) {
      ROS_WARN_THROTTLE(2.0, "Elevator-LIO adapter received an empty LiDAR cloud");
      return;
    }
    Eigen::Matrix4f transform = imu_t_lidar_.matrix().cast<float>();
    pcl::transformPointCloud(lidar_cloud, body_cloud, transform);

    CloudMsg output_cloud;
    pcl::toROSMsg(body_cloud, output_cloud);
    output_cloud.header = cloud->header;
    output_cloud.header.stamp = output_odom.header.stamp;
    output_cloud.header.frame_id = output_body_frame_;

    odom_pub_.publish(output_odom);
    cloud_pub_.publish(output_cloud);
  }

  void onElevatorFlag(const std_msgs::BoolConstPtr& msg) {
    std_msgs::Bool output;
    output.data = msg->data;
    elevator_flag_pub_.publish(output);
    if (!msg->data && last_elevator_mode_) {
      ROS_INFO("Elevator-LIO adapter: normal mode restored");
    } else if (msg->data && !last_elevator_mode_) {
      ROS_INFO("Elevator-LIO adapter: elevator mode entered");
    }
    last_elevator_mode_ = msg->data;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher odom_pub_;
  ros::Publisher cloud_pub_;
  ros::Publisher elevator_flag_pub_;
  message_filters::Subscriber<Odom> odom_sub_;
  message_filters::Subscriber<CloudMsg> cloud_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  ros::Subscriber elevator_flag_sub_;
  Eigen::Isometry3d imu_t_lidar_ = Eigen::Isometry3d::Identity();
  std::string output_odom_frame_;
  std::string output_body_frame_;
  int sync_queue_size_ = 30;
  bool elevator_flag_latched_ = true;
  bool last_elevator_mode_ = false;
};

}  // namespace dog_nav

int main(int argc, char** argv) {
  ros::init(argc, argv, "elevator_lio_adapter");
  try {
    dog_nav::ElevatorLioAdapter adapter;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Elevator-LIO adapter: %s", error.what());
    return 1;
  }
  return 0;
}
