#include "dog_nav_rviz_plugins/goal3d_tool.h"

#include <cmath>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rviz/display_context.h>
#include <rviz/properties/string_property.h>

namespace dog_nav_rviz
{
namespace
{
constexpr char kElevatorMarkTopic[] = "/elevator_mark/pose";
}

Goal3DTool::Goal3DTool()
{
  shortcut_key_ = 'g';
  topic_property_ = new rviz::StringProperty(
      "Topic",
      "/move_base_simple/goal",
      "The topic on which to publish 3D navigation goals.",
      getPropertyContainer(),
      &Goal3DTool::updateTopic,
      this);
}

void Goal3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  setName("3D Nav Goal");
  setArrowColor(1.0f, 0.15f, 0.85f);
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  const std::string topic = topic_property_->getStdString();
  if (nh_.resolveName(topic) == kElevatorMarkTopic)
  {
    pub_ = nh_.advertise<geometry_msgs::PoseArray>(topic, 1);
    setArrowColor(1.0f, 0.6f, 0.1f);
  }
  else
  {
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic, 1);
    setArrowColor(1.0f, 0.15f, 0.85f);
  }
}

void Goal3DTool::activate()
{
  Pose3DTool::activate();
  if (pub_.getTopic() == kElevatorMarkTopic)
  {
    setStatus("在候梯点或轿厢停车点按下，拖到同层门槛中点后松开。");
  }
}

bool Goal3DTool::onPoseSet(double x, double y, double z, double yaw, rviz::ViewportMouseEvent& event)
{
  if (pub_.getTopic() == kElevatorMarkTopic)
  {
    return publishElevatorMark(x, y, z, event);
  }
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  ROS_INFO("3D Nav Goal: (%.2f, %.2f, %.2f) yaw=%.2f -> %s",
           x, y, z, yaw, topic_property_->getStdString().c_str());
  pub_.publish(msg);
  return true;
}

bool Goal3DTool::publishElevatorMark(double x, double y, double z,
                                    rviz::ViewportMouseEvent& event)
{
  Ogre::Vector3 door;
  if (!pickPosition(event, door) || std::abs(door.z - z) > .4 ||
      std::hypot(door.x - x, door.y - y) < .2)
  {
    setStatus("请从位姿位置拖到同一层的门槛地面，再松开。");
    return false;
  }
  geometry_msgs::PoseArray mark;
  mark.header.frame_id = context_->getFixedFrame().toStdString();
  mark.header.stamp = ros::Time::now();
  mark.poses.resize(2);
  auto& pose = mark.poses.front();
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  const double yaw = std::atan2(door.y - y, door.x - x);
  pose.orientation.z = std::sin(yaw / 2.0);
  pose.orientation.w = std::cos(yaw / 2.0);
  auto& threshold = mark.poses.back();
  threshold.position.x = door.x;
  threshold.position.y = door.y;
  threshold.position.z = door.z;
  threshold.orientation.w = 1.0;
  pub_.publish(mark);
  return true;
}

}  // namespace dog_nav_rviz

PLUGINLIB_EXPORT_CLASS(dog_nav_rviz::Goal3DTool, rviz::Tool)
