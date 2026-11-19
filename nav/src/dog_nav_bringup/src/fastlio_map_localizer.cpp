// T_A_B maps points from B into A. FAST-LIO's "body" is the lidar IMU,
// not the robot base_link. Keep local odometry continuous and correct map->odom.
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <dog_nav_bringup/registration_checks.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <memory>
#include <thread>
#include <stdexcept>

using Transform = Eigen::Isometry3d;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
constexpr double kRad = 3.14159265358979323846 / 180.0;

static double yaw(const Transform& t) { return std::atan2(t(1, 0), t(0, 0)); }
static double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
static Transform fromPose(const geometry_msgs::Pose& p) {
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (!q.coeffs().allFinite() || q.norm() < 1e-6) throw std::runtime_error("invalid quaternion");
  Transform t = Transform::Identity();
  t.linear() = q.normalized().toRotationMatrix();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  if (!t.matrix().allFinite()) throw std::runtime_error("invalid position");
  return t;
}
static geometry_msgs::Pose toPose(const Transform& t) {
  geometry_msgs::Pose p;
  p.position.x = t.translation().x(); p.position.y = t.translation().y(); p.position.z = t.translation().z();
  Eigen::Quaterniond q(t.linear());
  p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z(); p.orientation.w = q.w();
  return p;
}
static Cloud::Ptr voxel(const Cloud::ConstPtr& cloud, double size) {
  Cloud::Ptr out(new Cloud);
  pcl::VoxelGrid<Point> filter;
  filter.setInputCloud(cloud); filter.setLeafSize(size, size, size); filter.filter(*out);
  return out;
}

class MapLocalizer {
 public:
  MapLocalizer() : nh_(), pnh_("~"), listener_(buffer_) {
    auto positive = [this](const char* name, double fallback) {
      double v; pnh_.param(name, v, fallback);
      if (!(v > 0) || !std::isfinite(v)) throw std::runtime_error(std::string("invalid parameter: ") + name);
      return v;
    };
    pnh_.param("input_odom_topic", input_odom_topic_, std::string("/fastlio_nav/odometry"));
    pnh_.param("input_cloud_topic", input_cloud_topic_, std::string("/fastlio_nav/cloud_body"));
    pnh_.param("input_odom_frame", input_odom_frame_, std::string("camera_init"));
    pnh_.param("input_body_frame", input_body_frame_, std::string("body"));
    pnh_.param("input_cloud_frame", input_cloud_frame_, std::string("body"));
    pnh_.param("map_frame", map_frame_, std::string("map"));
    pnh_.param("odom_tf_frame", odom_tf_frame_, std::string("fastlio_odom"));
    pnh_.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    pnh_.param("sensor_frame", sensor_frame_, std::string("livox_frame"));
    pnh_.param("elevator_mode_enabled", elevator_mode_enabled_, false);
    pnh_.param("elevator_flag_topic", elevator_flag_topic_, std::string("/LIO/in_elevator"));
    map_voxel_ = positive("map_voxel_size", .20);
    scan_voxel_ = positive("scan_voxel_size", .25);
    obstacle_voxel_ = positive("obstacle_voxel_size", .05);
    range_ = positive("max_scan_range", 25);
    radius_ = positive("submap_radius", 30);
    period_ = positive("registration_period", 1);
    tracking_range_ = positive("tracking_scan_range", 12);
    tracking_radius_ = positive("tracking_submap_radius", 16);
    submap_shift_ = positive("submap_update_distance", 3);
    verification_period_ = positive("pose_verification_period", .2);
    pnh_.param("pose_verification_max_points", verification_points_, 600);
    inlier_distance_ = positive("inlier_distance", .35);
    inlier_fraction_ = positive("min_inlier_fraction", .50);
    rmse_limit_ = positive("max_inlier_rmse", .25);
    verified_fraction_ = positive("verified_pose_min_inlier_fraction", .60);
    verified_rmse_ = positive("verified_pose_max_inlier_rmse", .20);
    initial_translation_ = positive("initial_max_translation", 2);
    initial_rotation_ = positive("initial_max_rotation_deg", 45) * kRad;
    tracking_translation_ = positive("tracking_max_translation", .35);
    elevator_reacquire_translation_ = positive("elevator_reacquire_max_translation", .85);
    tracking_rotation_ = positive("tracking_max_rotation_deg", 6) * kRad;
    tilt_limit_ = positive("max_gravity_tilt_deg", 12) * kRad;
    initial_fraction_ = positive("initial_min_inlier_fraction", .65);
    initial_rmse_ = positive("initial_max_inlier_rmse", .18);
    search_xy_ = positive("initial_search_xy", .75);
    search_z_ = positive("initial_search_z", .5);
    search_yaw_ = positive("initial_search_yaw_deg", 25) * kRad;
    pnh_.param("initial_confirmations", initial_confirmations_, 2);
    linear_rate_ = positive("correction_linear_rate", .05);
    angular_rate_ = positive("correction_angular_rate_deg", 2) * kRad;
    match_timeout_ = positive("global_match_timeout", 5);
    input_timeout_ = positive("input_timeout", .5);
    max_speed_ = positive("max_local_speed", 3);
    max_angular_ = positive("max_local_angular_speed", 3);
    pnh_.param("min_scan_points", min_points_, 150);
    pnh_.param("registration_threads", threads_, 2);
    pnh_.param("registration_iterations", iterations_, 40);
    if (verification_points_ < min_points_ || tracking_range_>range_ ||
        tracking_radius_<tracking_range_+submap_shift_+1. ||
        min_points_ < 20 || threads_ < 1 || iterations_ < 1 || inlier_fraction_ > 1 ||
        verified_fraction_ > 1 || verified_fraction_ < inlier_fraction_ || verified_rmse_ > rmse_limit_ ||
        initial_fraction_ > 1 || initial_fraction_ < inlier_fraction_ || initial_rmse_ > rmse_limit_ ||
        initial_confirmations_ < 2 || initial_confirmations_ > 5 || tilt_limit_ > 15*kRad ||
        search_xy_ > initial_translation_ || search_z_ > initial_translation_ || search_yaw_ > initial_rotation_)
      throw std::runtime_error("invalid registration parameters");
    std::vector<double> r, t, lo, hi;
    const bool private_extrinsic =
        pnh_.getParam("imu_lidar_rotation", r) && r.size() == 9 &&
        pnh_.getParam("imu_lidar_translation", t) && t.size() == 3;
    if (!private_extrinsic &&
        (!nh_.getParam("/fastlio_nav/mapping/extrinsic_R", r) || r.size() != 9 ||
         !nh_.getParam("/fastlio_nav/mapping/extrinsic_T", t) || t.size() != 3))
      throw std::runtime_error("missing LIO lidar-to-IMU calibration");
    for (int i = 0; i < 3; ++i) {
      imu_lidar_.translation()[i] = t[i];
      for (int j = 0; j < 3; ++j) imu_lidar_.linear()(i, j) = r[3 * i + j];
    }
    if (!imu_lidar_.matrix().allFinite() ||
        (imu_lidar_.linear().transpose() * imu_lidar_.linear() - Eigen::Matrix3d::Identity()).norm() > .001)
      throw std::runtime_error("invalid FAST-LIO extrinsic rotation");
    pnh_.param("camera_box_min", lo, std::vector<double>{-.18, -.04, -.015});
    pnh_.param("camera_box_max", hi, std::vector<double>{-.03, .10, .075});
    if (lo.size() != 3 || hi.size() != 3) throw std::runtime_error("invalid camera box");
    box_lo_ = Eigen::Vector3d(lo[0], lo[1], lo[2]);
    box_hi_ = Eigen::Vector3d(hi[0], hi[1], hi[2]);
    std::string path; pnh_.param<std::string>("globalmap_pcd", path, "");
    Cloud::Ptr raw(new Cloud), finite(new Cloud);
    if (pcl::io::loadPCDFile(path, *raw) != 0) throw std::runtime_error("cannot read PCD: " + path);
    for (const auto& p : *raw) if (pcl::isFinite(p)) finite->push_back(p);
    map_ = voxel(finite, map_voxel_);
    if (map_->size() < static_cast<size_t>(min_points_)) throw std::runtime_error("map too small");
    map_tree_.setInputCloud(map_);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/globalmap", 1, true);
    sensor_msgs::PointCloud2 map_msg; pcl::toROSMsg(*map_, map_msg);
    map_msg.header.frame_id = map_frame_; map_pub_.publish(map_msg);
    loaded_pub_ = nh_.advertise<std_msgs::String>("/map_manager/loaded_pcd", 1, true);
    std_msgs::String loaded; loaded.data = path; loaded_pub_.publish(loaded);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/dog_nav/odometry", 10);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/dog_nav/pose_3d", 10);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/navigation/obstacle_points", 2);
    sensor_pub_ = nh_.advertise<nav_msgs::Odometry>("/dog_nav/sensor_pose", 2);
    status_pub_ = nh_.advertise<std_msgs::String>("/dog_nav/localization_status", 1, true);
    cancel_pub_ = nh_.advertise<std_msgs::Empty>("/map_manager/cancel", 1);
    initial_sub_ = nh_.subscribe("/initialpose", 1, &MapLocalizer::initialPose, this);
    odom_sub_.subscribe(nh_, input_odom_topic_, 20);
    cloud_sub_.subscribe(nh_, input_cloud_topic_, 20);
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        nav_msgs::Odometry, sensor_msgs::PointCloud2>;
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(20), odom_sub_, cloud_sub_));
    sync_->registerCallback(boost::bind(&MapLocalizer::scan, this, boost::placeholders::_1, boost::placeholders::_2));
    if (elevator_mode_enabled_) {
      elevator_flag_sub_ = nh_.subscribe(
          elevator_flag_topic_, 1, &MapLocalizer::elevatorFlag, this);
    }
    status("WAITING_INITIAL_POSE");
    ROS_INFO("LIO map localizer loaded %zu points. Set 3D Pose Estimate; wait for TRACKING before a goal.", map_->size());
    worker_ = std::thread(&MapLocalizer::worker, this);
    watchdog_ = nh_.createTimer(ros::Duration(.2), &MapLocalizer::watchdog, this);
  }
  ~MapLocalizer() {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
    wake_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  // All shared state and status calls use mutex_; callbacks run on one ROS thread.
  void status(const std::string& value) {
    if (value == status_) return;
    status_ = value; std_msgs::String msg; msg.data = value; status_pub_.publish(msg);
    ROS_INFO_STREAM("LIO localization: " << value);
  }
  void initialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    if (msg->header.frame_id != map_frame_ && msg->header.frame_id != "/" + map_frame_) {
      ROS_ERROR("Initial pose must use %s frame", map_frame_.c_str()); return;
    }
    Transform pose;
    try { pose = fromPose(msg->pose.pose); }
    catch (const std::exception& e) { ROS_ERROR("Initial pose: %s", e.what()); return; }
    std::lock_guard<std::mutex> lock(mutex_);
    initial_pose_ = pose; have_initial_ = true; initialized_ = false; seeded_ = false;
    initial_matches_ = 0;
    ++generation_; latest_.reset(); last_match_ = ros::Time();
    local_submap_.reset();last_verification_=ros::Time();
    cancel_pub_.publish(std_msgs::Empty());
    status("ALIGNING");
  }
  bool mountTransform() {
    if (have_mount_) return true;
    try {
      auto tf = buffer_.lookupTransform("base_link", "livox_frame", ros::Time(0));
      geometry_msgs::Pose p;
      p.position.x = tf.transform.translation.x; p.position.y = tf.transform.translation.y; p.position.z = tf.transform.translation.z;
      p.orientation = tf.transform.rotation;
      // T_B_I = T_B_L * T_L_I. The physical mounting tilt is used exactly once.
      base_imu_ = fromPose(p) * imu_lidar_.inverse();
      have_mount_ = true; return true;
    } catch (const std::exception& e) {
      ROS_WARN_THROTTLE(3, "Waiting for base_link->livox_frame calibration: %s", e.what()); return false;
    }
  }
  void elevatorFlag(const std_msgs::BoolConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!elevator_mode_enabled_ || msg->data == elevator_mode_) return;
    elevator_mode_ = msg->data;
    ++generation_;  // Ignore registration jobs from the previous ride phase.
    elevator_matches_ = 0;
    target_ = current_;
    local_submap_.reset();
    latest_.reset();
    last_verification_ = ros::Time();
    if (elevator_mode_) {
      elevator_reacquire_pending_ = false;
      elevator_height_update_accepted_ = false;
      last_match_ = ros::Time::now();
      status(initialized_ ? "ELEVATOR_MODE" : "WAITING_INITIAL_POSE");
      ROS_INFO("LIO map localizer: elevator mode entered; GICP correction frozen");
    } else {
      elevator_reacquire_pending_ = initialized_;
      last_match_ = ros::Time::now();
      status(initialized_ ? "REACQUIRING_MAP" : "WAITING_INITIAL_POSE");
      ROS_INFO("LIO map localizer: normal mode restored; destination-floor map reacquisition requested");
    }
  }
  void scan(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& msg) {
    const ros::Time stamp = odom->header.stamp;
    const double age = (ros::Time::now() - stamp).toSec();
    if (age > input_timeout_ || age < -.2 || !mountTransform()) return;
    if (odom->header.frame_id != input_odom_frame_ ||
        odom->child_frame_id != input_body_frame_ ||
        msg->header.frame_id != input_cloud_frame_) {
      ROS_ERROR_THROTTLE(3, "Unexpected LIO frame names (expected %s/%s and cloud %s)",
                         input_odom_frame_.c_str(), input_body_frame_.c_str(),
                         input_cloud_frame_.c_str());
      return;
    }
    Transform local_imu;
    try { local_imu = fromPose(odom->pose.pose); } catch (const std::exception&) { return; }
    const Transform local_base = local_imu * base_imu_.inverse();
    const double dt = previous_stamp_.isZero() ? 0 : (stamp - previous_stamp_).toSec();
    Eigen::Vector3d linear = Eigen::Vector3d::Zero(), angular = Eigen::Vector3d::Zero();
    bool reset = false;
    Eigen::Vector3d displacement = Eigen::Vector3d::Zero();
    double rotation = 0.;
    if (!previous_stamp_.isZero()) {
      displacement = local_base.translation() - previous_base_.translation();
      Eigen::AngleAxisd turn(previous_base_.linear().transpose() * local_base.linear());
      rotation = std::abs(turn.angle());
      reset = dt <= 0 || displacement.norm() > max_speed_ * std::max(dt, .01) + .05 ||
              rotation > max_angular_ * std::max(dt, .01) + .05;
      if (dt > 0 && dt <= input_timeout_) {
        linear = local_base.linear().transpose() * (local_base.translation() - previous_base_.translation()) / dt;
        angular = local_base.linear().transpose() * previous_base_.linear() * turn.axis() * turn.angle() / dt;
      }
    }
    previous_base_ = local_base; previous_stamp_ = stamp;
    Cloud::Ptr raw(new Cloud), clean(new Cloud);
    pcl::fromROSMsg(*msg, *raw); clean->reserve(raw->size());
    const Transform lidar_imu = imu_lidar_.inverse();
    for (const auto& p : *raw) {
      if (!pcl::isFinite(p)) continue;
      Eigen::Vector3d lidar = lidar_imu * p.getVector3fMap().cast<double>();
      if (lidar.norm() > range_) continue;
      if ((lidar.array() >= box_lo_.array()).all() && (lidar.array() <= box_hi_.array()).all()) continue;
      clean->push_back(p);
    }
    Transform map_local;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (reset) {
        const Eigen::Vector3d map_delta = current_.linear() * displacement;
        // Exit ZUPT can commit a step in height before the elevator flag turns
        // false. Keep the map anchor and never treat this frame as a velocity.
        // The flag transition still requires a new successful map match.
        if (initialized_ && elevator_mode_ && !elevator_height_update_accepted_ &&
            dt > 0. && dt <= input_timeout_ && rotation <= .10 &&
            map_delta.head<2>().norm() <= .08 && std::abs(map_delta.z()) <= 1.0 &&
            std::abs(map_delta.z()) > max_speed_ * std::max(dt, .01) + .05) {
          elevator_height_update_accepted_ = true;
          last_input_ = stamp;
          ROS_INFO("Elevator-LIO ride height correction accepted: dz=%.3f m; map match required after exit",
                   map_delta.z());
          return;
        }
        ROS_WARN("LIO pose discontinuity: dt=%.3f s, map dxyz=(%.3f, %.3f, %.3f) m, rotation=%.3f rad, elevator=%d",
                 dt, map_delta.x(), map_delta.y(), map_delta.z(), rotation, elevator_mode_);
        initialized_ = have_initial_ = seeded_ = false; latest_.reset(); ++generation_;
        cancel_pub_.publish(std_msgs::Empty()); status("LIO_RESET_SET_INITIAL_POSE");
        return;
      }
      last_input_ = stamp;
      if (!have_initial_) return;
      if (!seeded_) {
        current_ = Transform::Identity();
        current_.linear() = Eigen::AngleAxisd(angle(yaw(initial_pose_) - yaw(local_base)), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        current_.translation() = initial_pose_.translation() - current_.linear() * local_base.translation();
        target_ = current_; seeded_ = true;
      }
      if (elevator_mode_enabled_ && elevator_mode_) {
        // Preserve the last map->LIO-world transform while Elevator-LIO carries
        // continuous world odometry through the shaft. Do not feed scans to GICP
        // and do not let map-match age suppress the navigation odometry.
        if (!initialized_) return;
        last_match_ = stamp;
        map_local = current_;
        status("ELEVATOR_MODE");
      } else {
        latest_ = clean; latest_pose_ = local_imu; latest_stamp_ = stamp;
        wake_.notify_one();
        if (!initialized_) return;
        // Bounded verification runs with the LIO callback, independently of a
        // potentially slow GICP solve. It queries ONLY the cached local submap.
        if(local_submap_ && (last_verification_.isZero() || (stamp-last_verification_).toSec()>=verification_period_)) {
          last_verification_=stamp;
          auto near=nearby(clean,tracking_range_);
          if(near->size()>=static_cast<size_t>(min_points_)) {
            const size_t stride=std::max<size_t>(1,(near->size()+verification_points_-1)/verification_points_);
            const auto evidence=quality(near,current_*local_imu,stride,local_submap_->verification_tree);
            if(evidence.fraction>=verified_fraction_&&evidence.rmse<=verified_rmse_ &&
               (ros::Time::now()-stamp).toSec()<=input_timeout_)
              last_match_=std::max(last_match_,stamp);
          }
        }
        if ((stamp - last_match_).toSec() > match_timeout_) { status("LOST_MAP_MATCH"); return; }
        // Smooth global corrections, never smooth or replace the local motion.
        const double step = std::max(0.0, std::min(dt, .2));
        // The saved map can have a small gravity-frame tilt. Preserve the full
        // validated SE(3) registration rather than discarding roll and pitch.
        // Limit correction at the current body anchor, so rotating map->odom
        // about a distant origin cannot produce an unbounded body-position jump.
        const Eigen::Vector3d local_anchor=local_base.translation();
        const Eigen::Vector3d current_anchor=current_*local_anchor;
        Eigen::Vector3d delta=target_*local_anchor-current_anchor;
        const double distance=delta.norm();
        if (distance>linear_rate_*step && distance>0.) delta*=linear_rate_*step/distance;
        const Eigen::Quaterniond current_rotation(current_.linear()),target_rotation(target_.linear());
        const double rotation_distance=current_rotation.angularDistance(target_rotation);
        const double fraction=rotation_distance>1e-12?std::min(1.,angular_rate_*step/rotation_distance):1.;
        current_.linear()=current_rotation.slerp(fraction,target_rotation).normalized().toRotationMatrix();
        current_.translation()=current_anchor+delta-current_.linear()*local_anchor;
        map_local = current_;
        if (!elevator_reacquire_pending_) status("TRACKING");
      }
    }
    nav_msgs::Odometry out;
    out.header.stamp = stamp; out.header.frame_id = map_frame_; out.child_frame_id = robot_base_frame_;
    out.pose.pose = toPose(map_local * local_base);
    out.twist.twist.linear.x = linear.x(); out.twist.twist.linear.y = linear.y(); out.twist.twist.linear.z = linear.z();
    out.twist.twist.angular.x = angular.x(); out.twist.twist.angular.y = angular.y(); out.twist.twist.angular.z = angular.z();
    odom_pub_.publish(out);
    geometry_msgs::PoseStamped body; body.header = out.header; body.pose = out.pose.pose; pose_pub_.publish(body);
    broadcast(map_local, map_frame_, odom_tf_frame_, stamp);
    broadcast(local_base, odom_tf_frame_, robot_base_frame_, stamp);
    const Transform map_imu = map_local * local_imu;
    nav_msgs::Odometry sensor; sensor.header = out.header;
    sensor.child_frame_id = sensor_frame_;
    sensor.pose.pose = toPose(map_imu * imu_lidar_); sensor_pub_.publish(sensor);
    Cloud::Ptr obstacles = voxel(clean, obstacle_voxel_);
    pcl::transformPointCloud(*obstacles, *obstacles, map_imu.matrix().cast<float>().eval());
    sensor_msgs::PointCloud2 cloud; pcl::toROSMsg(*obstacles, cloud); cloud.header = out.header; cloud_pub_.publish(cloud);
  }
  void broadcast(const Transform& t, const std::string& parent, const std::string& child, ros::Time stamp) {
    geometry_msgs::TransformStamped msg; msg.header.stamp = stamp; msg.header.frame_id = parent; msg.child_frame_id = child;
    auto pose = toPose(t); msg.transform.rotation = pose.orientation;
    msg.transform.translation.x = pose.position.x; msg.transform.translation.y = pose.position.y; msg.transform.translation.z = pose.position.z;
    broadcaster_.sendTransform(msg);
  }
  void watchdog(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_ && !last_input_.isZero() && (ros::Time::now() - last_input_).toSec() > input_timeout_) status("LOST_LIO_INPUT");
  }
  struct LocalSubmap {
    Cloud::Ptr points;
    pcl::KdTreeFLANN<Point> registration_tree,verification_tree;
    Eigen::Vector3d center;
    bool initial=false;
    unsigned epoch=0;
  };
  static Cloud::Ptr nearby(const Cloud::ConstPtr& source,double range) {
    Cloud::Ptr result(new Cloud);result->reserve(source->size());
    for(const auto& p:*source)if(p.getVector3fMap().squaredNorm()<=range*range)result->push_back(p);
    return result;
  }
  struct AlignmentQuality {double fraction,rmse;};
  AlignmentQuality quality(const Cloud::ConstPtr& source,const Transform& map_imu,size_t stride,
                           pcl::KdTreeFLANN<Point>& tree) {
    int inliers=0,samples=0;double sum=0.;
    std::vector<int> nearest(1);std::vector<float> squared(1);
    const auto rotation=map_imu.linear().cast<float>().eval();
    const auto translation=map_imu.translation().cast<float>().eval();
    for(size_t i=0;i<source->size();i+=stride) {
      Point p;p.getVector3fMap()=rotation*(*source)[i].getVector3fMap()+translation;++samples;
      if(tree.nearestKSearch(p,1,nearest,squared)&&squared[0]<inlier_distance_*inlier_distance_) {
        ++inliers;sum+=squared[0];
      }
    }
    return {samples?double(inliers)/samples:0.,inliers?std::sqrt(sum/inliers):1e9};
  }
  void worker() {
    ros::Time processed;
    auto next = std::chrono::steady_clock::now();
    std::shared_ptr<LocalSubmap> cached;
    fast_gicp::FastGICP<Point,Point> registration;
    registration.setNumThreads(threads_);registration.setMaximumIterations(iterations_);
    registration.setTransformationEpsilon(.001);
    while (ros::ok()) {
      Cloud::ConstPtr scan; Transform local_imu, guess, published; ros::Time stamp; bool initial, reacquiring; unsigned epoch;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait_for(lock, std::chrono::milliseconds(100), [this, &processed, &next] {
          return stopping_ || (latest_ && latest_stamp_ != processed && std::chrono::steady_clock::now() >= next);
        });
        if (stopping_) return;
        if (!latest_ || latest_stamp_ == processed || std::chrono::steady_clock::now() < next) continue;
        scan = latest_; local_imu = latest_pose_; stamp = latest_stamp_;
        guess = target_; published = current_; initial = !initialized_; epoch = generation_; processed = stamp;
        reacquiring = initialized_ && elevator_reacquire_pending_;
      }
      {
        bool frozen = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          frozen = elevator_mode_enabled_ && elevator_mode_;
        }
        if (frozen) continue;
      }
      next = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int>(1000 * period_));
      if ((ros::Time::now() - stamp).toSec() > input_timeout_) continue;
      Cloud::Ptr source = voxel(initial?scan:nearby(scan,tracking_range_),scan_voxel_);
      if (source->size() < static_cast<size_t>(min_points_)) continue;
      const Eigen::Vector3d center=(guess*local_imu).translation();
      if(!cached||cached->epoch!=epoch||cached->initial!=initial||
          (center-cached->center).norm()>submap_shift_) {
        Point anchor;anchor.getVector3fMap()=center.cast<float>();
        std::vector<int> indices;std::vector<float> distances;
        const double radius=initial?radius_:tracking_radius_;
        map_tree_.radiusSearch(anchor,radius,indices,distances);
        if(indices.size()<static_cast<size_t>(min_points_))continue;
        auto fresh=std::make_shared<LocalSubmap>();fresh->center=center;fresh->initial=initial;fresh->epoch=epoch;
        fresh->points.reset(new Cloud(*map_,indices));
        fresh->registration_tree.setInputCloud(fresh->points);
        fresh->verification_tree.setInputCloud(fresh->points);
        registration.setInputTarget(fresh->points);cached=fresh;
        {std::lock_guard<std::mutex> lock(mutex_);if(generation_==epoch)local_submap_=fresh;}
        ROS_INFO("LIO %s local submap: radius %.1f m, %zu/%zu map points; cached until %.1f m movement",
                 initial?"initialization":"tracking",radius,indices.size(),map_->size(),submap_shift_);
      }
      // A rejected optimizer proposal does not invalidate the current pose.
      // Independently check fresh scan/map agreement before retaining it.
      const auto retainVerifiedPose = [&](bool freeze_correction) {
        if (initial) return;
        const auto verified = quality(source,published*local_imu,1,cached->registration_tree);
        if (verified.fraction < verified_fraction_ || verified.rmse > verified_rmse_) return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (generation_ != epoch || !initialized_ || !have_initial_ ||
            (ros::Time::now() - stamp).toSec() > input_timeout_ ||
            (current_*local_imu.translation()-published*local_imu.translation()).norm() > .02 ||
            std::abs(Eigen::AngleAxisd(published.linear().transpose()*current_.linear()).angle()) > .5*kRad) return;
        // On rejection, freeze pending corrections; only this alignment was verified.
        if (freeze_correction) target_ = current_;
        last_match_ = std::max(last_match_,stamp);
        if (freeze_correction)
          ROS_WARN_THROTTLE(3, "LIO keeping verified current map alignment: %.1f%% overlap, RMSE %.3f m; rejected correction not applied",
                            verified.fraction * 100, verified.rmse);
      };
      // Verify the fresh scan BEFORE expensive GICP. Checking only after a
      // failed solve can consume the entire input freshness budget and discard
      // a valid alignment, eventually suppressing navigation odometry.
      // This refreshes map evidence only; successful correction logic is unchanged.
      retainVerifiedPose(false);
      registration.setMaxCorrespondenceDistance(initial?2.0:1.0);
      registration.setInputSource(source);
      const Transform anchor_pose=local_imu*base_imu_.inverse();
      const Transform bounds=initial?published:guess;
      const double max_shift = initial ? initial_translation_ :
          (reacquiring ? elevator_reacquire_translation_ : tracking_translation_);
      const double min_overlap = initial ? initial_fraction_ :
          (reacquiring ? std::max(.70, verified_fraction_) : inlier_fraction_);
      const double max_rmse = initial ? initial_rmse_ :
          (reacquiring ? std::min(.15, verified_rmse_) : rmse_limit_);
      Transform candidate=Transform::Identity();
      double fraction=0.,rmse=1e9,tilt=0.,shift=0.,rotation=0.;
      std::string rejection="limits_or_quality";
      auto solve=[&](const Transform& seed) {
        Cloud aligned;
        registration.align(aligned,(seed*local_imu).matrix().cast<float>().eval());
        if (!registration.hasConverged()){rejection="optimizer_not_converged";return false;}
        if (!registration.getFinalTransformation().allFinite()){rejection="nonfinite_transform";return false;}
        rejection="limits_or_quality";
        Transform map_imu=Transform::Identity();map_imu.matrix()=registration.getFinalTransformation().cast<double>();
        candidate=map_imu*local_imu.inverse();
        candidate.linear()=Eigen::Quaterniond(candidate.linear()).normalized().toRotationMatrix();
        tilt=dog_nav::registrationTilt(candidate);
        shift=dog_nav::registrationShift(bounds,candidate,anchor_pose.translation());
        rotation=dog_nav::registrationRotation(bounds,candidate);
        const auto q=quality(source,candidate*local_imu,1,cached->registration_tree);fraction=q.fraction;rmse=q.rmse;
        return tilt<=tilt_limit_ && shift<=max_shift &&
            rotation<=(initial?initial_rotation_:tracking_rotation_) &&
            fraction>=min_overlap && rmse<=max_rmse;
      };
      bool accepted=solve(guess);
      if (!accepted && initial) {
        // A mouse estimate can land on a floor, a wall or one of the separated
        // FREE points. Rank bounded XYZ/yaw hypotheses before refining a few;
        // do not keep retrying an identical GICP local minimum indefinitely.
        struct Seed { Transform pose;double score; };
        std::vector<Seed> seeds;
        const Eigen::Vector3d anchor=published*anchor_pose.translation();
        for (double dx:{-search_xy_,0.,search_xy_}) for (double dy:{-search_xy_,0.,search_xy_})
          for (double dz:{-search_z_,0.,search_z_}) for (double da:{-search_yaw_,0.,search_yaw_}) {
            Transform seed=published;
            seed.linear()=Eigen::AngleAxisd(da,Eigen::Vector3d::UnitZ()).toRotationMatrix()*published.linear();
            seed.translation()=anchor+Eigen::Vector3d(dx,dy,dz)-seed.linear()*anchor_pose.translation();
            const auto q=quality(source,seed*local_imu,std::max<size_t>(1,source->size()/500),cached->registration_tree);
            seeds.push_back({seed,q.fraction-.5*q.rmse});
          }
        std::sort(seeds.begin(),seeds.end(),[](const Seed& a,const Seed& b) { return a.score>b.score; });
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(2500);
        for (size_t i=0;i<std::min<size_t>(4,seeds.size());++i) {
          { std::lock_guard<std::mutex> lock(mutex_);if (stopping_ || generation_!=epoch) break; }
          if (std::chrono::steady_clock::now()>deadline) break;
          if (solve(seeds[i].pose)) { accepted=true;break; }
        }
      }
      if (!accepted) {
        if(rejection!="limits_or_quality")
          ROS_WARN_THROTTLE(3,"LIO local registration rejected (%s): candidate quality unavailable; %s",
                            rejection.c_str(),initial?"initial pose remains unconfirmed":"retaining only independently verified pose");
        else
          ROS_WARN_THROTTLE(3,"LIO local registration rejected: body shift %.3f m, rotation %.2f deg, map tilt %.2f/%.2f deg, overlap %.1f%%, RMSE %.3f m; %s",
                            shift,rotation/kRad,tilt/kRad,tilt_limit_/kRad,fraction*100,rmse,
                            initial?"adjust initial position/yaw if this persists":"keeping only independently verified pose");
        if (initial || reacquiring) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (generation_==epoch) {
            if (initial) { initial_matches_=0;target_=current_; }
            if (reacquiring) elevator_matches_=0;
          }
        }
        retainVerifiedPose(true);continue;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (generation_ != epoch || !have_initial_ || (ros::Time::now() - stamp).toSec() > match_timeout_) continue;
        if (initial) {
          const bool consistent=initial_matches_>0 &&
              dog_nav::registrationShift(target_,candidate,anchor_pose.translation())<=.10 &&
              dog_nav::registrationRotation(target_,candidate)<=2*kRad;
          initial_matches_=consistent?initial_matches_+1:1;
          target_=candidate;
          if (initial_matches_<initial_confirmations_) {
            ROS_INFO("LIO initial candidate verified: %.1f%%, RMSE %.3f m, tilt %.2f deg; checking another scan (%d/%d)",
                     fraction*100,rmse,tilt/kRad,initial_matches_,initial_confirmations_);
            continue;
          }
        }
        if (reacquiring) {
          // The chassis is held during reacquisition. Commit only after
          // distinct fresh scans agree, before making this pose executable.
          if ((ros::Time::now() - stamp).toSec() > std::min(2., match_timeout_)) {
            elevator_matches_ = 0;
            continue;
          }
          const bool consistent = elevator_matches_ > 0 &&
              dog_nav::registrationShift(elevator_candidate_, candidate, anchor_pose.translation()) <= .15 &&
              dog_nav::registrationRotation(elevator_candidate_, candidate) <= 3*kRad;
          elevator_matches_ = consistent ? elevator_matches_ + 1 : 1;
          elevator_candidate_ = candidate;
          if (elevator_matches_ < 2) {
            ROS_INFO("LIO exit map match candidate: shift %.3f m, %.1f%% overlap, RMSE %.3f m; waiting for another scan",
                     shift, fraction * 100., rmse);
            continue;
          }
          current_ = candidate;
          ROS_INFO("LIO exit map match accepted: shift %.3f m, %.1f%% overlap, RMSE %.3f m",
                   shift, fraction * 100., rmse);
        }
        target_ = candidate; last_match_ = std::max(last_match_,stamp);
        elevator_reacquire_pending_ = false;
        if (!initialized_) {
          current_ = candidate; initialized_ = true;
          ROS_INFO("LIO initial map match accepted: %.1f%%, RMSE %.3f m, full map-frame tilt %.2f deg retained",
                   fraction*100,rmse,tilt/kRad);
        }
        // TRACKING is published by the next fresh, synchronized LIO frame.
      }
    }
  }
  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer buffer_; tf2_ros::TransformListener listener_; tf2_ros::TransformBroadcaster broadcaster_;
  ros::Publisher map_pub_, loaded_pub_, odom_pub_, pose_pub_, cloud_pub_, sensor_pub_, status_pub_, cancel_pub_;
  ros::Subscriber initial_sub_, elevator_flag_sub_; ros::Timer watchdog_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  std::unique_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
      nav_msgs::Odometry, sensor_msgs::PointCloud2>>> sync_;
  Cloud::Ptr map_; pcl::KdTreeFLANN<Point> map_tree_;
  Transform imu_lidar_ = Transform::Identity(), base_imu_ = Transform::Identity();
  Transform previous_base_ = Transform::Identity(); ros::Time previous_stamp_; bool have_mount_ = false;
  Eigen::Vector3d box_lo_, box_hi_;
  double map_voxel_, scan_voxel_, obstacle_voxel_, range_, radius_, period_, inlier_distance_, inlier_fraction_, rmse_limit_;
  double tracking_range_,tracking_radius_,submap_shift_,verification_period_;
  int verification_points_=600;
  std::shared_ptr<LocalSubmap> local_submap_;
  ros::Time last_verification_;
  double verified_fraction_, verified_rmse_;
  double initial_fraction_,initial_rmse_,search_xy_,search_z_,search_yaw_;
  int initial_confirmations_=2,initial_matches_=0,elevator_matches_=0;
  double initial_translation_, initial_rotation_, tracking_translation_, tracking_rotation_, tilt_limit_;
  double elevator_reacquire_translation_;
  double linear_rate_, angular_rate_, match_timeout_, input_timeout_, max_speed_, max_angular_;
  int min_points_, threads_, iterations_;
  std::mutex mutex_; std::condition_variable wake_; std::thread worker_; bool stopping_ = false;
  bool have_initial_ = false, seeded_ = false, initialized_ = false; unsigned generation_ = 0;
  bool elevator_mode_enabled_ = false, elevator_mode_ = false, elevator_reacquire_pending_ = false;
  bool elevator_height_update_accepted_ = false;
  Transform initial_pose_ = Transform::Identity(), current_ = Transform::Identity(), target_ = Transform::Identity();
  Transform elevator_candidate_ = Transform::Identity();
  Cloud::ConstPtr latest_; Transform latest_pose_ = Transform::Identity();
  ros::Time latest_stamp_, last_match_, last_input_; std::string status_;
  std::string input_odom_topic_, input_cloud_topic_, elevator_flag_topic_;
  std::string input_odom_frame_, input_body_frame_, input_cloud_frame_;
  std::string map_frame_, odom_tf_frame_, robot_base_frame_, sensor_frame_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastlio_map_localizer");
  try { MapLocalizer node; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("FAST-LIO map localizer: %s", e.what()); return 1; }
  return 0;
}
