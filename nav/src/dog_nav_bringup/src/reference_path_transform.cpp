#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <dog_nav_bringup/planar_reference.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <stdexcept>

class ReferencePathTransform
{
public:
  ReferencePathTransform() : private_nh_("~"), listener_(ros::Duration(20.0))
  {
    private_nh_.param<std::string>("target_frame", target_frame_, "map");
    private_nh_.param("z_offset", z_offset_, 0.0);
    private_nh_.param("body_height", body_height_, .4);
    private_nh_.param("reference_offset_x", reference_offset_.x(), 0.0);
    private_nh_.param("reference_offset_y", reference_offset_.y(), 0.0);
    if (!reference_offset_.allFinite() || !std::isfinite(body_height_) || body_height_<=0.)
      throw std::invalid_argument("invalid planning reference configuration");
    std::string odom_topic;
    private_nh_.param<std::string>("odom_topic", odom_topic, "/dog_nav/odometry");
    odom_pub_=nh_.advertise<nav_msgs::Odometry>("/planning/body_odometry",20);
    odom_sub_=nh_.subscribe(odom_topic,20,&ReferencePathTransform::odomCallback,this);
    path_pub_ = nh_.advertise<nav_msgs::Path>("output", 1, true);
    path_sub_ = nh_.subscribe("input", 1, &ReferencePathTransform::pathCallback, this);
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr& input) {
    if (input->header.frame_id!=target_frame_) return;
    const auto& q=input->pose.pose.orientation;
    Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    Eigen::Vector3d point(input->pose.pose.position.x,input->pose.pose.position.y,input->pose.pose.position.z);
    if (!point.allFinite() || !rotation.coeffs().allFinite() || rotation.norm()<1e-9) return;
    rotation.normalize();const Eigen::Matrix3d matrix=rotation.toRotationMatrix();
    const double yaw=std::atan2(matrix(1,0),matrix(0,0));
    point=dog_nav::bodyReference(point,yaw,reference_offset_);
    nav_msgs::Odometry output=*input;
    output.child_frame_id="planning_body_reference";
    output.pose.pose.position.x=point.x();output.pose.pose.position.y=point.y();
    const auto& v=input->twist.twist.linear;const auto& w=input->twist.twist.angular;
    const Eigen::Vector3d omega=matrix*Eigen::Vector3d(w.x,w.y,w.z);
    const Eigen::Vector3d velocity=matrix.transpose()*dog_nav::bodyReferenceVelocity(
        matrix*Eigen::Vector3d(v.x,v.y,v.z),omega.z(),yaw,reference_offset_);
    if (!velocity.allFinite()) return;
    output.twist.twist.linear.x=velocity.x();output.twist.twist.linear.y=velocity.y();output.twist.twist.linear.z=velocity.z();
    odom_pub_.publish(output);
    body_start_=point;body_stamp_=input->header.stamp;
    if (pending_path_) { const auto path=pending_path_;pending_path_.reset();pathCallback(path); }
  }
  void pathCallback(const nav_msgs::PathConstPtr& input)
  {
    if (!input)
    {
      return;
    }

    if (input->poses.empty())
    {
      pending_path_.reset();
      nav_msgs::Path empty;
      empty.header.stamp = input->header.stamp.isZero() ? ros::Time::now() : input->header.stamp;
      empty.header.frame_id = target_frame_;
      path_pub_.publish(empty);
      return;
    }

    const double age=(ros::Time::now()-body_stamp_).toSec();
    if (body_stamp_.isZero() || age<-.1 || age>.5) {
      pending_path_=input;
      ROS_WARN_THROTTLE(1.,"[reference_path_transform] waiting for fresh body reference before accepting route");
      return;
    }

    const std::string source_frame = input->header.frame_id.empty()
                                         ? input->poses.front().header.frame_id
                                         : input->header.frame_id;
    if (source_frame.empty())
    {
      ROS_ERROR_THROTTLE(1.0, "[reference_path_transform] input path has no frame_id");
      return;
    }

    nav_msgs::Path output;
    // Preserve the goal identity used by completion events across the bridge.
    output.header.stamp = input->header.stamp.isZero() ? ros::Time::now() : input->header.stamp;
    output.header.frame_id = target_frame_;
    output.poses.reserve(input->poses.size());

    try
    {
      if (source_frame != target_frame_)
      {
        listener_.waitForTransform(target_frame_, source_frame, ros::Time(0), ros::Duration(1.0));
      }

      for (const auto& input_pose : input->poses)
      {
        geometry_msgs::PoseStamped source_pose = input_pose;
        if (source_pose.header.frame_id.empty())
        {
          source_pose.header.frame_id = source_frame;
        }
        source_pose.header.stamp = ros::Time(0);

        geometry_msgs::PoseStamped target_pose;
        if (source_pose.header.frame_id == target_frame_)
        {
          target_pose = source_pose;
        }
        else
        {
          listener_.transformPose(target_frame_, source_pose, target_pose);
        }
        target_pose.pose.position.z += z_offset_;
        target_pose.header.stamp = output.header.stamp;
        output.poses.push_back(target_pose);
      }
    }
    catch (const tf::TransformException& error)
    {
      ROS_ERROR("[reference_path_transform] cannot transform %s -> %s: %s",
                source_frame.c_str(),
                target_frame_.c_str(),
                error.what());
      return;
    }

    std::vector<Eigen::Vector3d> points;
    for (const auto& p:output.poses) points.emplace_back(p.pose.position.x,p.pose.position.y,p.pose.position.z);
    const auto& q=output.poses.back().pose.orientation;
    const Eigen::Quaterniond orientation(q.w,q.x,q.y,q.z);
    if (!orientation.coeffs().allFinite() || orientation.norm()<1e-9) return;
    const auto matrix=orientation.normalized().toRotationMatrix();
    const double yaw=std::atan2(matrix(1,0),matrix(0,0));
    Eigen::Vector3d start=body_start_;start.z()-=body_height_;
    points=dog_nav::bodyReferenceRoute(points,yaw,reference_offset_,start);
    const auto goal_orientation=output.poses.back().pose.orientation;
    output.poses.resize(points.size());
    for (size_t i=0;i<points.size();++i) {
      output.poses[i].pose.position.x=points[i].x();output.poses[i].pose.position.y=points[i].y();output.poses[i].pose.position.z=points[i].z();
    }
    output.poses.back().pose.orientation=goal_orientation;
    path_pub_.publish(output);
    ROS_INFO("[reference_path_transform] published %zu points in %s",
             output.poses.size(),
             target_frame_.c_str());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf::TransformListener listener_;
  ros::Subscriber path_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher odom_pub_;
  nav_msgs::PathConstPtr pending_path_;
  Eigen::Vector2d reference_offset_=Eigen::Vector2d::Zero();
  Eigen::Vector3d body_start_=Eigen::Vector3d::Zero();
  ros::Time body_stamp_;
  ros::Publisher path_pub_;
  std::string target_frame_;
  double z_offset_;
  double body_height_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reference_path_transform");
  ReferencePathTransform node;
  ros::spin();
  return 0;
}
