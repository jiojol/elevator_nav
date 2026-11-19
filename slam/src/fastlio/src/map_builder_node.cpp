#include <map>
#include <algorithm>
#include <mutex>
#include <vector>
#include <thread>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>

#ifndef FASTLIO_WITH_GTSAM
#define FASTLIO_WITH_GTSAM 0
#endif

#if FASTLIO_WITH_GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/linearExceptions.h>
#endif

#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver2/CustomMsg.h>

#include "lio_builder/lio_builder.h"
#include "fastlio/SaveMap.h"
#include "localizer/icp_localizer.h"

#if FASTLIO_WITH_GTSAM
class ZaxisPriorFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
    double z_;

public:
    ZaxisPriorFactor(gtsam::Key key, const gtsam::SharedNoiseModel &noise, double z)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise, key), z_(z)
    {
    }
    virtual ~ZaxisPriorFactor()
    {
    }
    virtual gtsam::Vector evaluateError(const gtsam::Pose3 &p, boost::optional<gtsam::Matrix &> H = boost::none) const
    {
        auto z = p.translation()(2);
        if (H)
        {
            gtsam::Matrix Jac = gtsam::Matrix::Zero(1, 6);
            Jac << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
            (*H) = Jac;
        }
        return gtsam::Vector1(z - z_);
    }
};
#endif

bool terminate_flag = false;

void signalHandler(int signum)
{
    std::cout << "SHUTTING DOWN MAPPING NODE!" << std::endl;
    terminate_flag = true;
    (void)signum;
}

struct LoopPair
{
    LoopPair(int p, int c, float s, Eigen::Matrix3d &dr, Eigen::Vector3d &dp) : pre_idx(p), cur_idx(c), score(s), diff_rot(dr), diff_pos(dp) {}
    int pre_idx;
    int cur_idx;
    Eigen::Matrix3d diff_rot;
    Eigen::Vector3d diff_pos;
    double score;
};

struct Pose6D
{
    Pose6D(int i, double t, Eigen::Matrix3d lr, Eigen::Vector3d lp) : index(i), time(t), local_rot(lr), local_pos(lp) {}
    void setGlobalPose(const Eigen::Matrix3d &gr, const Eigen::Vector3d &gp)
    {
        global_rot = gr;
        global_pos = gp;
    }
    void addOffset(const Eigen::Matrix3d &offset_rot, const Eigen::Vector3d &offset_pos)
    {
        global_rot = offset_rot * local_rot;
        global_pos = offset_rot * local_pos + offset_pos;
    }

    void getOffset(Eigen::Matrix3d &offset_rot, Eigen::Vector3d &offset_pos)
    {
        offset_rot = global_rot * local_rot.transpose();
        offset_pos = -global_rot * local_rot.transpose() * local_pos + global_pos;
    }
    int index;
    double time;
    Eigen::Matrix3d local_rot;
    Eigen::Vector3d local_pos;
    Eigen::Matrix3d global_rot;
    Eigen::Vector3d global_pos;
    // Eigen::Vector3d gravity;
};

struct SharedData
{
    bool key_pose_added = false;
    std::mutex mutex;
    Eigen::Matrix3d offset_rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d offset_pos = Eigen::Vector3d::Zero();
    std::vector<Pose6D> key_poses;
    std::vector<LoopPair> loop_pairs;
    std::vector<std::pair<int, int>> loop_history;
    std::vector<fastlio::PointCloudXYZI::Ptr> cloud_history;
    uint64_t optimization_revision = 0;
};

struct MapVoxelKey
{
    int64_t x;
    int64_t y;
    int64_t z;

    bool operator==(const MapVoxelKey &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct MapVoxelKeyHash
{
    size_t operator()(const MapVoxelKey &key) const
    {
        size_t seed = std::hash<int64_t>{}(key.x);
        seed ^= std::hash<int64_t>{}(key.y) + 0x9e3779b97f4a7c15ULL +
                (seed << 6) + (seed >> 2);
        seed ^= std::hash<int64_t>{}(key.z) + 0x9e3779b97f4a7c15ULL +
                (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct MapVoxelAccumulator
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double intensity = 0.0;
    uint32_t count = 0;
};

struct LoopParams
{
    double rad_thresh = 0.4;
    double dist_thresh = 2.5;
    double time_thresh = 30.0;
    double loop_pose_search_radius = 10.0;
    int loop_pose_index_thresh = 5;
    double submap_resolution = 0.2;
    int submap_search_num = 20;
    double loop_icp_thresh = 0.3;
    double loop_noise_scale = 1.0;
    int loop_accept_interval = 30;
    bool force_final_to_start = false;
    double final_loop_max_distance = 10.0;
    bool activate = true;
};

#if FASTLIO_WITH_GTSAM
class LoopClosureThread
{
public:
    void init()
    {
        gtsam::ISAM2Params isam2_params;
        isam2_params.relinearizeThreshold = 0.01;
        isam2_params.relinearizeSkip = 1;
        isam2_ = std::make_shared<gtsam::ISAM2>(isam2_params);
        kdtree_history_poses_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        cloud_history_poses_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        sub_map_downsize_filter_.reset(new pcl::VoxelGrid<fastlio::PointType>);
        sub_map_downsize_filter_->setLeafSize(loop_params_.submap_resolution,
                                              loop_params_.submap_resolution,
                                              loop_params_.submap_resolution);

        icp_.reset(new pcl::IterativeClosestPointWithNormals<fastlio::PointType, fastlio::PointType>);
        icp_->setMaxCorrespondenceDistance(100);
        icp_->setMaximumIterations(50);
        icp_->setTransformationEpsilon(1e-6);
        icp_->setEuclideanFitnessEpsilon(1e-6);
        icp_->setRANSACIterations(0);
    }
    void setShared(std::shared_ptr<SharedData> share_data)
    {
        shared_data_ = share_data;
    }
    void setRate(const double &rate)
    {
        rate_ = std::make_shared<ros::WallRate>(rate);
    }
    LoopParams &mutableParams()
    {
        return loop_params_;
    }

    void finalize()
    {
        if (!loop_params_.activate || !loop_params_.force_final_to_start)
            return;

        {
            std::lock_guard<std::mutex> lock(shared_data_->mutex);
            if (shared_data_->key_poses.size() < 2)
                return;
            temp_poses_.assign(shared_data_->key_poses.begin(), shared_data_->key_poses.end());
        }

        lastest_index_ = static_cast<int>(temp_poses_.size()) - 1;
        const double endpoint_distance =
            (temp_poses_.back().global_pos - temp_poses_.front().global_pos).norm();
        if (endpoint_distance > loop_params_.final_loop_max_distance)
        {
            ROS_WARN("Skip final-to-start constraint: endpoint distance %.3f m exceeds %.3f m",
                     endpoint_distance, loop_params_.final_loop_max_distance);
            return;
        }

        addOdomFactor();
        // Same physical start/end pose: constrain both translation and yaw.
        // Previous code used a weak rotation prior (1e6) which closed XYZ but
        // left ~50 deg yaw error and warped the map.
        const gtsam::Pose3 final_to_start_measurement = gtsam::Pose3();
        const auto final_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.15, 0.15, 0.15, 0.20, 0.20, 0.20).finished());
        gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            0, lastest_index_, final_to_start_measurement, final_noise));
        loop_found_ = true;
        if (!smoothAndUpdate())
        {
            ROS_WARN("Skip final-to-start result because GTSAM optimization failed.");
            return;
        }
        shared_data_->loop_history.emplace_back(0, lastest_index_);
        ROS_INFO("Applied final-to-start SE3 constraint: 0 %d, prior distance %.3f m",
                 lastest_index_, endpoint_distance);
    }

    fastlio::PointCloudXYZI::Ptr getSubMaps(std::vector<Pose6D> &pose_list,
                                            std::vector<fastlio::PointCloudXYZI::Ptr> &cloud_list,
                                            int index,
                                            int search_num)
    {
        fastlio::PointCloudXYZI::Ptr cloud(new fastlio::PointCloudXYZI);
        int max_size = pose_list.size();
        int min_index = std::max(0, index - search_num);
        int max_index = std::min(max_size - 1, index + search_num);
        for (int i = min_index; i <= max_index; i++)
        {
            Pose6D &p = pose_list[i];
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) = p.global_rot;
            T.block<3, 1>(0, 3) = p.global_pos;
            fastlio::PointCloudXYZI::Ptr temp_cloud(new fastlio::PointCloudXYZI);
            pcl::transformPointCloud(*cloud_list[p.index], *temp_cloud, T);
            *cloud += *temp_cloud;
        }
        sub_map_downsize_filter_->setInputCloud(cloud);
        sub_map_downsize_filter_->filter(*cloud);
        return cloud;
    }

    void operator()()
    {
        while (ros::ok())
        {
            rate_->sleep();
            if (terminate_flag)
                break;
            if (!loop_params_.activate)
                continue;
            if (shared_data_->key_poses.size() < loop_params_.loop_pose_index_thresh)
                continue;
            if (!shared_data_->key_pose_added)
                continue;
            shared_data_->key_pose_added = false;
            {
                std::lock_guard<std::mutex> lock(shared_data_->mutex);
                lastest_index_ = shared_data_->key_poses.size() - 1;
                temp_poses_.clear();
                temp_poses_.assign(shared_data_->key_poses.begin(), shared_data_->key_poses.end());
            }

            loopCheck();
            if (!loop_found_)
                continue;
            addOdomFactor();
            addLoopFactor();
            smoothAndUpdate();
        }
    }

private:
    std::shared_ptr<SharedData> shared_data_;

    std::shared_ptr<ros::WallRate> rate_;

    LoopParams loop_params_;

    std::vector<Pose6D> temp_poses_;

    int previous_index_ = 0;

    int lastest_index_;

    bool loop_found_ = false;

    int last_accepted_loop_index_ = -1000000;

    gtsam::Values initialized_estimate_;

    gtsam::Values optimized_estimate_;

    std::shared_ptr<gtsam::ISAM2> isam2_;

    gtsam::NonlinearFactorGraph gtsam_graph_;

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_history_poses_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_history_poses_;

    pcl::VoxelGrid<fastlio::PointType>::Ptr sub_map_downsize_filter_;

    pcl::IterativeClosestPointWithNormals<fastlio::PointType, fastlio::PointType>::Ptr icp_;

    void loopCheck()
    {
        if (temp_poses_.empty())
            return;
        int cur_index = temp_poses_.size() - 1;
        int pre_index = -1;

        cloud_history_poses_->clear();

        for (Pose6D &p : temp_poses_)
        {
            pcl::PointXYZ point;
            point.x = p.global_pos(0);
            point.y = p.global_pos(1);
            point.z = p.global_pos(2);
            cloud_history_poses_->push_back(point);
        }
        kdtree_history_poses_->setInputCloud(cloud_history_poses_);
        std::vector<int> ids;
        std::vector<float> sqdists;
        kdtree_history_poses_->radiusSearch(cloud_history_poses_->back(), loop_params_.loop_pose_search_radius, ids, sqdists, 0);

        for (int i = 0; i < ids.size(); i++)
        {
            int id = ids[i];
            if (std::abs(temp_poses_[id].time - temp_poses_.back().time) > loop_params_.time_thresh)
            {
                pre_index = id;
                break;
            }
        }
        if (pre_index == -1 || pre_index == cur_index || cur_index - pre_index < loop_params_.loop_pose_index_thresh)
            return;
        if (cur_index - last_accepted_loop_index_ < loop_params_.loop_accept_interval)
            return;

        fastlio::PointCloudXYZI::Ptr cur_cloud = getSubMaps(temp_poses_, shared_data_->cloud_history, cur_index, 0);
        fastlio::PointCloudXYZI::Ptr sub_maps = getSubMaps(temp_poses_, shared_data_->cloud_history, pre_index, loop_params_.submap_search_num);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud_xyz(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr sub_maps_xyz(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cur_cloud, *cur_cloud_xyz);
        pcl::copyPointCloud(*sub_maps, *sub_maps_xyz);
        cur_cloud = fastlio::IcpLocalizer::addNorm(cur_cloud_xyz);
        sub_maps = fastlio::IcpLocalizer::addNorm(sub_maps_xyz);

        icp_->setInputSource(cur_cloud);
        icp_->setInputTarget(sub_maps);

        fastlio::PointCloudXYZI::Ptr aligned(new fastlio::PointCloudXYZI);

        icp_->align(*aligned, Eigen::Matrix4f::Identity());

        float score = icp_->getFitnessScore();

        if (!icp_->hasConverged() || score > loop_params_.loop_icp_thresh)
            return;

        ROS_INFO("Detected LOOP: %d %d %f", pre_index, cur_index, score);
        shared_data_->loop_history.emplace_back(pre_index, cur_index);
        last_accepted_loop_index_ = cur_index;
        loop_found_ = true;

        Eigen::Matrix4d T_pre_cur = icp_->getFinalTransformation().cast<double>();
        Eigen::Matrix3d R12 = temp_poses_[pre_index].global_rot.transpose() * T_pre_cur.block<3, 3>(0, 0) * temp_poses_[cur_index].global_rot;
        Eigen::Vector3d t12 = temp_poses_[pre_index].global_rot.transpose() * (T_pre_cur.block<3, 3>(0, 0) * temp_poses_[cur_index].global_pos + T_pre_cur.block<3, 1>(0, 3) - temp_poses_[pre_index].global_pos);
        shared_data_->loop_pairs.emplace_back(pre_index, cur_index, score, R12, t12);
    }

    void addOdomFactor()
    {
        for (int i = previous_index_; i < lastest_index_; i++)
        {
            Pose6D &p1 = temp_poses_[i];
            Pose6D &p2 = temp_poses_[i + 1];

            if (i == 0)
            {
                initialized_estimate_.insert(i, gtsam::Pose3(gtsam::Rot3(p1.local_rot),
                                                             gtsam::Point3(p1.local_pos)));
                gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
                gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(i, gtsam::Pose3(gtsam::Rot3(p1.local_rot), gtsam::Point3(p1.local_pos)), noise));
            }
            initialized_estimate_.insert(i + 1, gtsam::Pose3(gtsam::Rot3(p2.local_rot),
                                                             gtsam::Point3(p2.local_pos)));
            Eigen::Matrix3d R12 = p1.local_rot.transpose() * p2.local_rot;
            Eigen::Vector3d t12 = p1.local_rot.transpose() * (p2.local_pos - p1.local_pos);
            gtsam::noiseModel::Diagonal::shared_ptr z_noise =
                gtsam::noiseModel::Diagonal::Variances(gtsam::Vector1::Ones() * 1e-6);
            gtsam_graph_.add(ZaxisPriorFactor(i + 1, z_noise, p2.local_pos(2)));

            gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
            gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(i,
                                                                i + 1,
                                                                gtsam::Pose3(gtsam::Rot3(R12), gtsam::Point3(t12)), noise));
        }
        previous_index_ = lastest_index_;
    }

    void addLoopFactor()
    {
        if (!loop_found_)
            return;
        if (shared_data_->loop_pairs.empty())
            return;
        for (LoopPair &lp : shared_data_->loop_pairs)
        {
            gtsam::Pose3 pose_between(gtsam::Rot3(lp.diff_rot), gtsam::Point3(lp.diff_pos));
            const double loop_variance =
                std::max(1e-4, lp.score * loop_params_.loop_noise_scale);
            gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(lp.pre_idx,
                                                                lp.cur_idx,
                                                                pose_between,
                                                                gtsam::noiseModel::Diagonal::Variances(
                                                                    gtsam::Vector6::Ones() * loop_variance)));
        }
        shared_data_->loop_pairs.clear();
    }

    bool smoothAndUpdate()
    {
        try
        {
            isam2_->update(gtsam_graph_, initialized_estimate_);
            isam2_->update();
            if (loop_found_)
            {
                isam2_->update();
                isam2_->update();
                isam2_->update();
                isam2_->update();
                isam2_->update();
                loop_found_ = false;
            }
            gtsam_graph_.resize(0);
            initialized_estimate_.clear();

            optimized_estimate_ = isam2_->calculateBestEstimate();
            gtsam::Pose3 latest_estimate = optimized_estimate_.at<gtsam::Pose3>(lastest_index_);
            temp_poses_[lastest_index_].global_rot = latest_estimate.rotation().matrix().cast<double>();
            temp_poses_[lastest_index_].global_pos = latest_estimate.translation().matrix().cast<double>();
            Eigen::Matrix3d offset_rot;
            Eigen::Vector3d offset_pos;
            temp_poses_[lastest_index_].getOffset(offset_rot, offset_pos);

            {
                std::lock_guard<std::mutex> lock(shared_data_->mutex);
                const int current_size = shared_data_->key_poses.size();
                shared_data_->offset_rot = offset_rot;
                shared_data_->offset_pos = offset_pos;

                for (int i = 0; i < lastest_index_; i++)
                {
                    gtsam::Pose3 temp_pose = optimized_estimate_.at<gtsam::Pose3>(i);
                    shared_data_->key_poses[i].global_rot =
                        temp_pose.rotation().matrix().cast<double>();
                    shared_data_->key_poses[i].global_pos =
                        temp_pose.translation().matrix().cast<double>();
                }

                for (int i = lastest_index_; i < current_size; i++)
                {
                    shared_data_->key_poses[i].addOffset(offset_rot, offset_pos);
                }
                ++shared_data_->optimization_revision;
            }
            return true;
        }
        catch (const gtsam::IndeterminantLinearSystemException &e)
        {
            ROS_ERROR_STREAM("GTSAM update skipped: indeterminate linear system near variable "
                             << e.nearbyVariable() << ". " << e.what());
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM("GTSAM update skipped: " << e.what());
        }
        resetGtsamBackend();
        return false;
    }

    void resetGtsamBackend()
    {
        gtsam::ISAM2Params isam2_params;
        isam2_params.relinearizeThreshold = 0.01;
        isam2_params.relinearizeSkip = 1;
        isam2_ = std::make_shared<gtsam::ISAM2>(isam2_params);
        gtsam_graph_.resize(0);
        initialized_estimate_.clear();
        optimized_estimate_ = gtsam::Values();
        previous_index_ = 0;
        loop_found_ = false;
        shared_data_->loop_pairs.clear();
    }
};
#else
class LoopClosureThread
{
public:
    void init()
    {
        if (loop_params_.activate)
        {
            ROS_WARN("GTSAM is not available; loop closure backend is disabled.");
            loop_params_.activate = false;
        }
    }

    void setShared(std::shared_ptr<SharedData> share_data)
    {
        shared_data_ = share_data;
    }

    void setRate(const double &rate)
    {
        rate_ = std::make_shared<ros::WallRate>(rate);
    }

    LoopParams &mutableParams()
    {
        return loop_params_;
    }

    void finalize()
    {
    }

    void operator()()
    {
        while (ros::ok())
        {
            if (terminate_flag)
                break;
            if (rate_)
                rate_->sleep();
            else
                ros::Duration(0.01).sleep();
        }
    }

private:
    std::shared_ptr<SharedData> shared_data_;
    std::shared_ptr<ros::WallRate> rate_;
    LoopParams loop_params_;
};
#endif

class MapBuilderROS
{
public:
    MapBuilderROS(tf2_ros::TransformBroadcaster &br, std::shared_ptr<SharedData> share_data) : br_(br)
    {
        shared_data_ = share_data;
        initPatams();
        initSubscribers();
        initPublishers();
        initServices();
        lio_builder_ = std::make_shared<fastlio::LIOBuilder>(lio_params_);
        if (offline_mode_)
        {
            loop_closure_.setRate(500.0);
        }
        else
        {
            loop_closure_.setRate(loop_rate_hz_);
        }
        loop_closure_.setShared(share_data);
        loop_closure_.init();
        loop_thread_ = std::make_shared<std::thread>(std::ref(loop_closure_));
    }

    void initPatams()
    {
        nh_.param<std::string>("map_frame", global_frame_, "map");
        nh_.param<std::string>("local_frame", local_frame_, "local");
        nh_.param<std::string>("body_frame", body_frame_, "body");
        nh_.param<std::string>("imu_topic", imu_data_.topic, "/livox/imu");
        nh_.param<std::string>("livox_topic", livox_data_.topic, "/livox/lidar");
        nh_.param<std::string>("lidar_type", lidar_type_, "pointcloud2");
        nh_.param<double>("time_offset_lidar_to_imu", livox_data_.time_offset_lidar_to_imu, 0.0);
        nh_.param<double>("pointcloud/blind", livox_data_.blind, 0.5);
        nh_.param<int>("pointcloud/filter_num", livox_data_.filter_num, 3);
        nh_.param<bool>("pointcloud/use_timestamp", livox_data_.use_point_timestamp, false);
        nh_.param<bool>("pointcloud/normalize_timestamp_segments",
                        livox_data_.normalize_timestamp_segments, false);
        nh_.param<bool>("pointcloud/sync_dual_lidar_segments",
                        livox_data_.sync_dual_lidar_segments, false);
        nh_.param<std::string>("pointcloud/timestamp_field",
                               livox_data_.timestamp_field, "timestamp");
        nh_.param<double>("pointcloud/timestamp_scale_to_sec",
                          livox_data_.timestamp_scale_to_sec, 1.0);
        nh_.param<double>("pointcloud/timestamp_backward_jump_sec",
                          livox_data_.timestamp_backward_jump_sec, 0.01);
        nh_.param<int>("pointcloud/max_timestamp_segments",
                       livox_data_.max_timestamp_segments, 4);
        nh_.param<double>("pointcloud/fallback_scan_period_sec",
                          livox_data_.fallback_scan_period_sec, 0.1);
        nh_.param<double>("pointcloud/dual_sync_tolerance_sec",
                          livox_data_.dual_sync_tolerance_sec, 0.002);
        nh_.param<double>("pointcloud/dual_hold_sec",
                          livox_data_.dual_hold_sec, 0.2);
        int dual_max_pending = 4;
        nh_.param<int>("pointcloud/dual_max_pending", dual_max_pending, 4);
        livox_data_.dual_max_pending =
            static_cast<size_t>(std::max(1, dual_max_pending));
        int max_output_buffer_frames = 32;
        nh_.param<int>("pointcloud/max_output_buffer_frames",
                       max_output_buffer_frames, 32);
        livox_data_.max_output_buffer_frames =
            static_cast<size_t>(std::max(1, max_output_buffer_frames));
        nh_.param<double>("pointcloud/dual_min_segment_duration_sec",
                          livox_data_.dual_min_segment_duration_sec, 0.08);
        nh_.param<double>("pointcloud/dual_max_segment_duration_sec",
                          livox_data_.dual_max_segment_duration_sec, 0.12);
        nh_.param<double>("pointcloud/dual_idle_flush_wall_sec",
                          livox_data_.dual_idle_flush_wall_sec, 1.0);
        nh_.param<bool>("pointcloud/dual_enable_idle_flush",
                        livox_data_.dual_enable_idle_flush, true);
        livox_data_.filter_num = std::max(1, livox_data_.filter_num);
        livox_data_.max_timestamp_segments =
            std::max(1, livox_data_.max_timestamp_segments);
        nh_.param<int>("pointcloud/subscriber_queue_size",
                       lidar_sub_queue_size_, 10);
        lidar_sub_queue_size_ = std::max(1, lidar_sub_queue_size_);
        nh_.param<bool>("adaptive_rate", adaptive_rate_, true);
        nh_.param<bool>("offline_mode", offline_mode_, false);
        nh_.param<bool>("offline_save_map", offline_save_map_, true);
        nh_.param<std::string>("offline_bag_path", offline_bag_path_, "");
        nh_.param<std::string>("offline_traj_output", offline_traj_output_, "/tmp/fastlio_traj.tum");
        nh_.param<std::string>("offline_map_output", offline_map_output_, "/tmp/fastlio_map.pcd");
        nh_.param<double>("offline_start_offset_sec",
                          offline_start_offset_sec_, 0.0);
        nh_.param<double>("offline_duration_sec", offline_duration_sec_, -1.0);
        offline_start_offset_sec_ = std::max(0.0, offline_start_offset_sec_);
        nh_.param<bool>("global_map/publish", publish_global_map_, false);
        nh_.param<int>("global_map/keypose_interval", global_map_keypose_interval_, 30);
        nh_.param<double>("global_map/resolution", global_map_resolution_, 0.25);
        nh_.param<bool>("keyframe/store_clouds", store_keyframe_clouds_, true);
        nh_.param<double>("keyframe/cloud_resolution",
                          keyframe_cloud_resolution_, 0.3);
        nh_.param<double>("save_map/resolution", save_map_resolution_, 0.3);
        nh_.param<int>("save_map/max_voxels", save_map_max_voxels_, 3000000);
        nh_.param<int>("save_map/max_resolution_attempts",
                       save_map_max_resolution_attempts_, 5);
        save_map_max_voxels_ = std::max(10000, save_map_max_voxels_);
        save_map_max_resolution_attempts_ =
            std::max(1, save_map_max_resolution_attempts_);
        double local_rate, loop_rate;
        nh_.param<double>("local_rate", local_rate, 20.0);
        nh_.param<double>("loop_rate", loop_rate, 1.0);
        local_rate_ = std::make_shared<ros::Rate>(local_rate);
        loop_rate_hz_ = loop_rate;
        nh_.param<double>("lio_builder/det_range", lio_params_.det_range, 100.0);
        nh_.param<double>("lio_builder/cube_len", lio_params_.cube_len, 500.0);
        nh_.param<double>("lio_builder/resolution", lio_params_.resolution, 0.1);
        nh_.param<double>("lio_builder/move_thresh", lio_params_.move_thresh, 1.5);
        nh_.param<double>("lio_builder/ikdtree/delete_criterion",
                          lio_params_.ikdtree_delete_criterion, 0.5);
        nh_.param<double>("lio_builder/ikdtree/balance_criterion",
                          lio_params_.ikdtree_balance_criterion, 0.6);
        nh_.param<bool>("lio_builder/ikdtree/async_rebuild_enabled",
                        lio_params_.ikdtree_async_rebuild_enabled, true);
        nh_.param<int>("lio_builder/ikdtree/async_rebuild_min_points",
                       lio_params_.ikdtree_async_rebuild_min_points,
                       Multi_Thread_Rebuild_Point_Num);
        nh_.param<int>("lio_builder/ikdtree/async_rebuild_max_points",
                       lio_params_.ikdtree_async_rebuild_max_points, 0);
        nh_.param<bool>("lio_builder/ikdtree/limit_delete_rebuild_above_max",
                        lio_params_.ikdtree_limit_delete_rebuild_above_max,
                        false);
        nh_.param<bool>("lio_builder/ikdtree/release_rebuild_storage",
                        lio_params_.ikdtree_release_rebuild_storage, true);
        nh_.param<bool>("lio_builder/ikdtree/print_rebuild_diagnostics",
                        lio_params_.ikdtree_print_rebuild_diagnostics, false);
        nh_.param<int>("lio_builder/ikdtree/diagnostic_interval_frames",
                       lio_params_.ikdtree_diagnostic_interval_frames, 0);
        nh_.param<bool>("lio_builder/align_gravity", lio_params_.align_gravity, true);
        nh_.param<double>("lio_builder/imu_acc_cov", lio_params_.imu_acc_cov, 0.01);
        nh_.param<double>("lio_builder/imu_gyro_cov", lio_params_.imu_gyro_cov, 0.01);
        nh_.param<double>("lio_builder/imu_acc_bias_cov", lio_params_.imu_acc_bias_cov, 0.0001);
        nh_.param<double>("lio_builder/imu_gyro_bias_cov", lio_params_.imu_gyro_bias_cov, 0.0001);
        nh_.param<int>("lio_builder/imu_init_samples",
                       lio_params_.imu_init_samples, 20);
        nh_.param<std::vector<double>>("lio_builder/imu_ext_rot", lio_params_.imu_ext_rot, std::vector<double>());
        nh_.param<std::vector<double>>("lio_builder/imu_ext_pos", lio_params_.imu_ext_pos, std::vector<double>());
        nh_.param<bool>("hdl_tunnel/directional_constraint",
                        lio_params_.hdl_directional_constraint, false);
        nh_.param<double>("hdl_tunnel/hessian_ratio_enter",
                          lio_params_.hdl_hessian_ratio_enter, 0.05);
        nh_.param<double>("hdl_tunnel/hessian_ratio_recover",
                          lio_params_.hdl_hessian_ratio_recover, 0.08);

        nh_.param<bool>("loop_closure/activate", loop_closure_.mutableParams().activate, true);
        nh_.param<double>("loop_closure/rad_thresh", loop_closure_.mutableParams().rad_thresh, 0.4);
        nh_.param<double>("loop_closure/dist_thresh", loop_closure_.mutableParams().dist_thresh, 2.5);
        nh_.param<double>("loop_closure/time_thresh", loop_closure_.mutableParams().time_thresh, 30.0);
        nh_.param<double>("loop_closure/loop_pose_search_radius", loop_closure_.mutableParams().loop_pose_search_radius, 10.0);
        nh_.param<int>("loop_closure/loop_pose_index_thresh", loop_closure_.mutableParams().loop_pose_index_thresh, 5);
        nh_.param<double>("loop_closure/submap_resolution", loop_closure_.mutableParams().submap_resolution, 0.2);
        nh_.param<int>("loop_closure/submap_search_num", loop_closure_.mutableParams().submap_search_num, 20);
        nh_.param<double>("loop_closure/loop_icp_thresh", loop_closure_.mutableParams().loop_icp_thresh, 0.3);
        nh_.param<double>("loop_closure/loop_noise_scale", loop_closure_.mutableParams().loop_noise_scale, 1.0);
        nh_.param<int>("loop_closure/loop_accept_interval", loop_closure_.mutableParams().loop_accept_interval, 30);
        nh_.param<bool>("loop_closure/force_final_to_start", loop_closure_.mutableParams().force_final_to_start, false);
        nh_.param<double>("loop_closure/final_loop_max_distance", loop_closure_.mutableParams().final_loop_max_distance, 10.0);
        if (loop_closure_.mutableParams().activate && !store_keyframe_clouds_)
        {
            ROS_WARN("loop closure requires keyframe clouds; overriding keyframe/store_clouds=true");
            store_keyframe_clouds_ = true;
        }
        nh_.param<bool>("odom/use_wheel_odom", lio_params_.use_wheel_odom, false);
        nh_.param<bool>("odom/fuse_velocity", lio_params_.odom_fuse_velocity, true);
        nh_.param<bool>("odom/fuse_z_velocity",
                        lio_params_.odom_fuse_z_velocity, true);
        nh_.param<bool>("odom/fuse_integrated_position",
                        lio_params_.odom_fuse_integrated_position, false);
        nh_.param<bool>("odom/fuse_angular_velocity",
                        lio_params_.odom_fuse_angular_velocity, false);
        nh_.param<bool>("odom/velocity_in_body_frame",
                        lio_params_.odom_velocity_in_body_frame, true);
        nh_.param<double>("odom/max_linear_speed",
                          lio_params_.odom_max_linear_speed, 3.0);
        nh_.param<double>("odom/max_angular_speed",
                          lio_params_.odom_max_angular_speed, 4.0);
        nh_.param<double>("odom/linear_deadband",
                          lio_params_.odom_linear_deadband, 0.01);
        nh_.param<double>("odom/angular_deadband",
                          lio_params_.odom_angular_deadband, 0.02);
        nh_.param<std::string>("odom/odom_topic", lio_params_.wheel_odom_topic, std::string("/wheel_odom"));
        nh_.param<std::vector<double>>("odom/R_odom_to_imu", lio_params_.R_odom_to_imu, std::vector<double>());
        nh_.param<double>("odom/pos_noise_std", lio_params_.odom_pos_noise_std, 0.05);
        nh_.param<double>("odom/vel_noise_std", lio_params_.odom_vel_noise_std, 0.1);
        nh_.param<double>("odom/rot_noise_std", lio_params_.odom_rot_noise_std, 0.08);
        nh_.param<double>("odom/rot_innovation_gate", lio_params_.odom_rot_innovation_gate, 0.05);
        nh_.param<double>("odom/rot_covariance_floor_scale",
                          lio_params_.odom_rot_covariance_floor_scale, 0.0);
        nh_.param<double>("odom/covariance_floor_scale", lio_params_.odom_covariance_floor_scale, 0.0);
        nh_.param<double>("odom/z_pos_noise_std", lio_params_.odom_z_pos_noise_std, 0.08);
        nh_.param<double>("odom/z_vel_noise_std", lio_params_.odom_z_vel_noise_std, 0.08);
        nh_.param<bool>("odom/use_orientation", lio_params_.odom_use_orientation, true);
        nh_.param<bool>("odom/use_absolute_orientation",
                        lio_params_.odom_use_absolute_orientation, false);
        nh_.param<bool>("odom/planar", lio_params_.odom_planar, false);
        nh_.param<bool>("odom/constrain_z", lio_params_.odom_constrain_z, false);
        nh_.param<bool>("odom/hard_constrain_z", lio_params_.odom_hard_constrain_z, true);
        nh_.param<bool>("odom/debug", lio_params_.odom_debug, false);
        ROS_INFO("Mapping lidar=%s (%s), imu=%s, time_offset_lidar_to_imu=%.8f, loop=%s, wheel_odom=%s topic=%s",
                 livox_data_.topic.c_str(), lidar_type_.c_str(), imu_data_.topic.c_str(),
                 livox_data_.time_offset_lidar_to_imu,
                 loop_closure_.mutableParams().activate ? "on" : "off",
                 lio_params_.use_wheel_odom ? "on" : "off",
                 lio_params_.wheel_odom_topic.c_str());
    }

    void initSubscribers()
    {
        imu_sub_ = nh_.subscribe(imu_data_.topic, 1000, &ImuData::callback, &imu_data_);
        if (lidar_type_ == "livox_custom")
        {
            livox_sub_ = nh_.subscribe<livox_ros_driver2::CustomMsg>(
                livox_data_.topic, lidar_sub_queue_size_,
                static_cast<void (LivoxData::*)(const livox_ros_driver2::CustomMsg::ConstPtr &)>(&LivoxData::callback),
                &livox_data_);
        }
        else
        {
            livox_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
                livox_data_.topic, lidar_sub_queue_size_,
                static_cast<void (LivoxData::*)(const sensor_msgs::PointCloud2::ConstPtr &)>(&LivoxData::callback),
                &livox_data_);
        }
        if (lio_params_.use_wheel_odom)
        {
            wheel_odom_sub_ = nh_.subscribe(lio_params_.wheel_odom_topic, 5000, &MapBuilderROS::wheelOdomCallback, this);
        }
    }

    void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (!lio_params_.use_wheel_odom)
            return;
        lio_builder_->pushWheelOdom(msg);
    }

    void initPublishers()
    {
        local_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("local_cloud", 10);
        body_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("body_cloud", 10);
        global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map", 1, true);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("slam_odom", 20);
        loop_mark_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("loop_mark", 10);

        local_path_pub_ = nh_.advertise<nav_msgs::Path>("local_path", 10);
        global_path_pub_ = nh_.advertise<nav_msgs::Path>("global_path", 10);
    }

    void initServices()
    {
        save_map_server_ = nh_.advertiseService("save_map", &MapBuilderROS::saveMapCallback, this);
    }

    void publishCloud(ros::Publisher &publisher, const sensor_msgs::PointCloud2 &cloud_to_pub)
    {
        if (offline_mode_)
            return;
        if (publisher.getNumSubscribers() == 0)
            return;
        publisher.publish(cloud_to_pub);
    }

    fastlio::PointCloudXYZI::Ptr makeKeyframeCloud()
    {
        if (!store_keyframe_clouds_)
            return fastlio::PointCloudXYZI::Ptr();

        fastlio::PointCloudXYZI::Ptr source = lio_builder_->cloudDownBody();
        if (!source || source->empty() || keyframe_cloud_resolution_ <= 0.0)
            return source;

        fastlio::PointCloudXYZI::Ptr filtered(new fastlio::PointCloudXYZI);
        pcl::VoxelGrid<fastlio::PointType> voxel_filter;
        const float resolution = static_cast<float>(keyframe_cloud_resolution_);
        voxel_filter.setLeafSize(resolution, resolution, resolution);
        voxel_filter.setInputCloud(source);
        voxel_filter.filter(*filtered);
        return filtered;
    }

    void recordTrajectoryPose()
    {
        Pose6D pose(static_cast<int>(trajectory_.size()), current_time_,
                    current_state_.rot.toRotationMatrix(), current_state_.pos);
        pose.addOffset(shared_data_->offset_rot, shared_data_->offset_pos);
        trajectory_.push_back(pose);
    }

    bool buildSparseVoxelMap(double requested_resolution,
                             fastlio::PointCloudXYZI::Ptr &cloud,
                             double &used_resolution,
                             std::string &error)
    {
        std::vector<Pose6D> poses;
        std::vector<fastlio::PointCloudXYZI::Ptr> clouds;
        {
            std::lock_guard<std::mutex> lock(shared_data_->mutex);
            poses = shared_data_->key_poses;
            clouds = shared_data_->cloud_history;
        }

        if (poses.empty() || clouds.empty())
        {
            error = "No keyframe clouds are available (keyframe/store_clouds may be false).";
            return false;
        }

        used_resolution = std::max(0.02, requested_resolution);
        for (int attempt = 0; attempt < save_map_max_resolution_attempts_; ++attempt)
        {
            std::unordered_map<MapVoxelKey, MapVoxelAccumulator, MapVoxelKeyHash> voxels;
            voxels.reserve(static_cast<size_t>(
                std::min(save_map_max_voxels_, 1000000)));
            bool limit_reached = false;

            for (const Pose6D &pose : poses)
            {
                if (pose.index < 0 || pose.index >= static_cast<int>(clouds.size()) ||
                    !clouds[pose.index])
                    continue;

                const Eigen::Matrix3d &rot = pose.global_rot;
                const Eigen::Vector3d &pos = pose.global_pos;
                for (const fastlio::PointType &point : clouds[pose.index]->points)
                {
                    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                        !std::isfinite(point.z))
                        continue;
                    const Eigen::Vector3d world =
                        rot * Eigen::Vector3d(point.x, point.y, point.z) + pos;
                    const MapVoxelKey key{
                        static_cast<int64_t>(std::floor(world.x() / used_resolution)),
                        static_cast<int64_t>(std::floor(world.y() / used_resolution)),
                        static_cast<int64_t>(std::floor(world.z() / used_resolution))};
                    auto it = voxels.find(key);
                    if (it == voxels.end())
                    {
                        if (voxels.size() >= static_cast<size_t>(save_map_max_voxels_))
                        {
                            limit_reached = true;
                            break;
                        }
                        it = voxels.emplace(key, MapVoxelAccumulator()).first;
                    }
                    MapVoxelAccumulator &acc = it->second;
                    acc.x += world.x();
                    acc.y += world.y();
                    acc.z += world.z();
                    acc.intensity += point.intensity;
                    ++acc.count;
                }
                if (limit_reached)
                    break;
            }

            if (limit_reached)
            {
                const double next_resolution = used_resolution * 1.5;
                ROS_WARN("Sparse map exceeded %d voxels at %.3f m; retrying at %.3f m.",
                         save_map_max_voxels_, used_resolution, next_resolution);
                used_resolution = next_resolution;
                continue;
            }

            cloud.reset(new fastlio::PointCloudXYZI);
            cloud->reserve(voxels.size());
            for (const auto &entry : voxels)
            {
                const MapVoxelAccumulator &acc = entry.second;
                if (acc.count == 0)
                    continue;
                fastlio::PointType point;
                const double inv_count = 1.0 / static_cast<double>(acc.count);
                point.x = static_cast<float>(acc.x * inv_count);
                point.y = static_cast<float>(acc.y * inv_count);
                point.z = static_cast<float>(acc.z * inv_count);
                point.intensity = static_cast<float>(acc.intensity * inv_count);
                point.normal_x = 0.0f;
                point.normal_y = 0.0f;
                point.normal_z = 0.0f;
                point.curvature = 0.0f;
                cloud->push_back(point);
            }
            cloud->width = static_cast<uint32_t>(cloud->size());
            cloud->height = 1;
            cloud->is_dense = true;
            if (cloud->empty())
            {
                error = "All keyframe clouds were empty or invalid.";
                return false;
            }
            return true;
        }

        error = "Voxel limit still exceeded after all resolution retries.";
        return false;
    }

    bool writeSparseVoxelMap(const std::string &path,
                             double requested_resolution,
                             double &used_resolution,
                             size_t &point_count,
                             std::string &error)
    {
        fastlio::PointCloudXYZI::Ptr cloud;
        if (!buildSparseVoxelMap(requested_resolution, cloud, used_resolution, error))
            return false;
        point_count = cloud->size();
        const int result = writer_.writeBinaryCompressed(path, *cloud);
        if (result != 0)
        {
            error = "PCD writer failed with code " + std::to_string(result) + ".";
            return false;
        }
        return true;
    }

    void publishGlobalMap()
    {
        if (offline_mode_ || !publish_global_map_ || global_map_pub_.getNumSubscribers() == 0)
            return;

        std::vector<Pose6D> poses;
        std::vector<fastlio::PointCloudXYZI::Ptr> clouds;
        uint64_t optimization_revision;
        {
            std::lock_guard<std::mutex> lock(shared_data_->mutex);
            const size_t keypose_count = shared_data_->key_poses.size();
            const bool interval_due =
                last_global_map_keypose_count_ == 0 ||
                keypose_count >= last_global_map_keypose_count_ +
                                     static_cast<size_t>(std::max(1, global_map_keypose_interval_));
            const bool optimization_due =
                shared_data_->optimization_revision != last_global_map_optimization_revision_;
            if (!interval_due && !optimization_due)
                return;

            poses = shared_data_->key_poses;
            clouds = shared_data_->cloud_history;
            optimization_revision = shared_data_->optimization_revision;
        }

        // Keep the online global map on the same bounded-memory sparse voxel
        // path as map saving.  This avoids PCL VoxelGrid integer overflow on a
        // long tunnel and avoids materializing several full-map copies.
        fastlio::PointCloudXYZI::Ptr filtered;
        double used_resolution = global_map_resolution_;
        std::string error;
        if (!buildSparseVoxelMap(global_map_resolution_, filtered,
                                 used_resolution, error))
        {
            ROS_WARN_THROTTLE(5.0, "Skip global map: %s", error.c_str());
            return;
        }
        global_map_pub_.publish(pcl2msg(filtered, global_frame_, current_time_));
        last_global_map_keypose_count_ = poses.size();
        last_global_map_optimization_revision_ = optimization_revision;
    }

    void publishOdom(const nav_msgs::Odometry &odom_to_pub)
    {
        if (offline_mode_)
            return;
        if (odom_pub_.getNumSubscribers() == 0)
            return;
        odom_pub_.publish(odom_to_pub);
    }

    void publishLocalPath()
    {
        if (offline_mode_)
            return;
        if (local_path_pub_.getNumSubscribers() == 0)
            return;

        if (shared_data_->key_poses.empty())
            return;

        nav_msgs::Path path;
        path.header.frame_id = global_frame_;
        path.header.stamp = ros::Time().fromSec(current_time_);
        for (Pose6D &p : shared_data_->key_poses)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = global_frame_;
            pose.header.stamp = ros::Time().fromSec(current_time_);
            pose.pose.position.x = p.local_pos(0);
            pose.pose.position.y = p.local_pos(1);
            pose.pose.position.z = p.local_pos(2);
            Eigen::Quaterniond q(p.local_rot);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            path.poses.push_back(pose);
        }
        local_path_pub_.publish(path);
    }

    void publishGlobalPath()
    {
        if (offline_mode_)
            return;
        if (global_path_pub_.getNumSubscribers() == 0)
            return;

        if (shared_data_->key_poses.empty())
            return;
        nav_msgs::Path path;
        path.header.frame_id = global_frame_;
        path.header.stamp = ros::Time().fromSec(current_time_);
        for (Pose6D &p : shared_data_->key_poses)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = global_frame_;
            pose.header.stamp = ros::Time().fromSec(current_time_);
            pose.pose.position.x = p.global_pos(0);
            pose.pose.position.y = p.global_pos(1);
            pose.pose.position.z = p.global_pos(2);
            Eigen::Quaterniond q(p.global_rot);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            path.poses.push_back(pose);
        }
        global_path_pub_.publish(path);
    }

    void publishLoopMark()
    {
        if (offline_mode_)
            return;
        if (loop_mark_pub_.getNumSubscribers() == 0)
            return;
        if (shared_data_->loop_history.empty())
            return;
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker nodes_marker;

        nodes_marker.header.frame_id = global_frame_;
        nodes_marker.header.stamp = ros::Time().fromSec(current_time_);
        nodes_marker.ns = "loop_nodes";
        nodes_marker.id = 0;
        nodes_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        nodes_marker.action = visualization_msgs::Marker::ADD;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.3;
        nodes_marker.scale.y = 0.3;
        nodes_marker.scale.z = 0.3;
        nodes_marker.color.r = 1.0;
        nodes_marker.color.g = 0.8;
        nodes_marker.color.b = 0.0;
        nodes_marker.color.a = 1.0;

        visualization_msgs::Marker edges_marker;
        edges_marker.header.frame_id = global_frame_;
        edges_marker.header.stamp = ros::Time().fromSec(current_time_);
        edges_marker.ns = "loop_edges";
        edges_marker.id = 1;
        edges_marker.type = visualization_msgs::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::Marker::ADD;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.1;

        edges_marker.color.r = 0.0;
        edges_marker.color.g = 0.8;
        edges_marker.color.b = 0.0;
        edges_marker.color.a = 1.0;
        for (auto &p : shared_data_->loop_history)
        {
            Pose6D &p1 = shared_data_->key_poses[p.first];
            Pose6D &p2 = shared_data_->key_poses[p.second];
            geometry_msgs::Point point1;
            point1.x = p1.global_pos(0);
            point1.y = p1.global_pos(1);
            point1.z = p1.global_pos(2);
            geometry_msgs::Point point2;
            point2.x = p2.global_pos(0);
            point2.y = p2.global_pos(1);
            point2.z = p2.global_pos(2);
            nodes_marker.points.push_back(point1);
            nodes_marker.points.push_back(point2);
            edges_marker.points.push_back(point1);
            edges_marker.points.push_back(point2);
        }
        marker_array.markers.push_back(nodes_marker);
        marker_array.markers.push_back(edges_marker);
        loop_mark_pub_.publish(marker_array);
    }

    bool saveMapCallback(fastlio::SaveMap::Request &req, fastlio::SaveMap::Response &res)
    {
        const double requested_resolution =
            req.resolution > 0.0 ? req.resolution : save_map_resolution_;
        double used_resolution = requested_resolution;
        size_t point_count = 0;
        std::string error;
        if (!writeSparseVoxelMap(req.save_path, requested_resolution,
                                 used_resolution, point_count, error))
        {
            res.status = 0;
            res.message = "Save map failed: " + error;
            ROS_ERROR("%s", res.message.c_str());
            return true;
        }
        res.status = 1;
        res.message = "Save map success: " + std::to_string(point_count) +
                      " points at " + std::to_string(used_resolution) + " m.";
        ROS_INFO("%s Path: %s", res.message.c_str(), req.save_path.c_str());
        return true;
    }

    void addKeyPose()
    {
        int idx = shared_data_->key_poses.size();
        if (shared_data_->key_poses.empty())
        {
            std::lock_guard<std::mutex> lock(shared_data_->mutex);
            shared_data_->key_poses.emplace_back(idx, current_time_, current_state_.rot.toRotationMatrix(), current_state_.pos);
            shared_data_->key_poses.back().addOffset(shared_data_->offset_rot, shared_data_->offset_pos);
            // shared_data_->key_poses.back().gravity = current_state_.get_g();
            shared_data_->key_pose_added = true;
            shared_data_->cloud_history.push_back(makeKeyframeCloud());
            return;
        }
        Pose6D &last_key_pose = shared_data_->key_poses.back();
        Eigen::Matrix3d diff_rot = last_key_pose.local_rot.transpose() * current_state_.rot.toRotationMatrix();
        Eigen::Vector3d diff_pose = last_key_pose.local_rot.transpose() * (current_state_.pos - last_key_pose.local_pos);
        Eigen::Vector3d rpy = rotate2rpy(diff_rot);
        if (diff_pose.norm() > loop_closure_.mutableParams().dist_thresh ||
            std::abs(rpy(0)) > loop_closure_.mutableParams().rad_thresh ||
            std::abs(rpy(1)) > loop_closure_.mutableParams().rad_thresh ||
            std::abs(rpy(2)) > loop_closure_.mutableParams().rad_thresh)
        {
            std::lock_guard<std::mutex> lock(shared_data_->mutex);
            shared_data_->key_poses.emplace_back(idx, current_time_, current_state_.rot.toRotationMatrix(), current_state_.pos);
            shared_data_->key_poses.back().addOffset(shared_data_->offset_rot, shared_data_->offset_pos);
            // shared_data_->key_poses.back().gravity = current_state_.get_g();
            shared_data_->key_pose_added = true;
            shared_data_->cloud_history.push_back(makeKeyframeCloud());
        }
    }

    void run()
    {
        if (offline_mode_)
        {
            runOffline();
            return;
        }
        ros::AsyncSpinner spinner(2);
        spinner.start();
        while (ros::ok())
        {
            if (terminate_flag)
                break;
            if (!measure_group_.syncPackage(imu_data_, livox_data_))
            {
                if (adaptive_rate_)
                {
                    // Avoid busy waiting while still reacting to incoming bag data quickly.
                    ros::WallDuration(0.0005).sleep();
                }
                else if (local_rate_)
                {
                    local_rate_->sleep();
                }
                continue;
            }
            lio_builder_->mapping(measure_group_);
            if (lio_builder_->currentStatus() == fastlio::Status::INITIALIZE)
                continue;
            current_time_ = measure_group_.lidar_time_end;
            current_state_ = lio_builder_->currentState();
            recordTrajectoryPose();
            br_.sendTransform(eigen2Transform(shared_data_->offset_rot,
                                               shared_data_->offset_pos,
                                               global_frame_,
                                               local_frame_,
                                               current_time_));
            br_.sendTransform(eigen2Transform(current_state_.rot.toRotationMatrix(),
                                               current_state_.pos,
                                               local_frame_,
                                               body_frame_,
                                               current_time_));

            publishOdom(eigen2Odometry(current_state_.rot.toRotationMatrix(),
                                       current_state_.pos,
                                       local_frame_,
                                       body_frame_,
                                       current_time_));

            addKeyPose();

            publishCloud(body_cloud_pub_,
                         pcl2msg(lio_builder_->cloudUndistortedBody(),
                                 body_frame_,
                                 current_time_));
            publishCloud(local_cloud_pub_,
                         pcl2msg(lio_builder_->cloudWorld(),
                                 local_frame_,
                                 current_time_));
            publishLocalPath();
            publishGlobalPath();
            publishLoopMark();
            publishGlobalMap();
        }

        terminate_flag = true;
        loop_thread_->join();
        finalizeAndSave();
        spinner.stop();
        std::cout << "MAPPING NODE IS DOWN!" << std::endl;
    }

    void runOffline()
    {
        ros::AsyncSpinner spinner(2);
        spinner.start();
        if (offline_bag_path_.empty())
        {
            ROS_ERROR("offline_mode is true but offline_bag_path is empty.");
            terminate_flag = true;
            loop_thread_->join();
            spinner.stop();
            return;
        }

        rosbag::Bag bag;
        try
        {
            bag.open(offline_bag_path_, rosbag::bagmode::Read);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Failed to open bag: %s", e.what());
            terminate_flag = true;
            loop_thread_->join();
            spinner.stop();
            return;
        }

        std::vector<std::string> topics{imu_data_.topic, livox_data_.topic};
        if (lio_params_.use_wheel_odom)
            topics.push_back(lio_params_.wheel_odom_topic);
        rosbag::View all_messages(bag);
        const ros::Time view_start =
            all_messages.getBeginTime() + ros::Duration(offline_start_offset_sec_);
        const ros::Time view_end = offline_duration_sec_ > 0.0
                                       ? view_start + ros::Duration(offline_duration_sec_)
                                       : all_messages.getEndTime();
        rosbag::View view(bag, rosbag::TopicQuery(topics), view_start, view_end);
        uint64_t total_msgs = view.size();
        if (total_msgs == 0)
        {
            ROS_WARN("Offline bag has no messages on imu/lidar/odom topics.");
        }
        else
        {
            ROS_INFO("Offline bag opened, total messages on selected topics: %lu", total_msgs);
        }
        uint64_t processed_msgs = 0;
        int last_progress = -1;
        bool started_processing = false;

        for (const rosbag::MessageInstance &m : view)
        {
            if (terminate_flag)
                break;
            if (m.getTopic() == imu_data_.topic || ("/" + imu_data_.topic) == m.getTopic())
            {
                sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
                if (imu)
                    imu_data_.callback(imu);
            }
            else if (m.getTopic() == livox_data_.topic || ("/" + livox_data_.topic) == m.getTopic())
            {
                livox_ros_driver2::CustomMsg::ConstPtr lidar = m.instantiate<livox_ros_driver2::CustomMsg>();
                if (lidar)
                {
                    livox_data_.callback(lidar);
                }
                else
                {
                    sensor_msgs::PointCloud2::ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2>();
                    if (cloud)
                        livox_data_.callback(cloud);
                }
            }
            else if (lio_params_.use_wheel_odom && (m.getTopic() == lio_params_.wheel_odom_topic || ("/" + lio_params_.wheel_odom_topic) == m.getTopic()))
            {
                nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();
                if (odom)
                    lio_builder_->pushWheelOdom(odom);
            }

            while (measure_group_.syncPackage(imu_data_, livox_data_))
            {
                if (!started_processing)
                {
                    started_processing = true;
                    ROS_INFO("Offline data synchronized, start processing...");
                }
                lio_builder_->mapping(measure_group_);
                if (lio_builder_->currentStatus() == fastlio::Status::INITIALIZE)
                    continue;
                current_time_ = measure_group_.lidar_time_end;
                current_state_ = lio_builder_->currentState();
                recordTrajectoryPose();
                addKeyPose();
            }

            ++processed_msgs;
            if (total_msgs > 0)
            {
                int progress = static_cast<int>((processed_msgs * 100) / total_msgs);
                if (progress >= last_progress + 5)
                {
                    ROS_INFO("Offline processing progress: %d%% (%lu/%lu)", progress, processed_msgs, total_msgs);
                    last_progress = progress;
                }
            }
        }

        if (livox_data_.sync_dual_lidar_segments)
        {
            std::lock_guard<std::mutex> lock(livox_data_.mutex);
            livox_data_.processDualPending(true);
            livox_data_.logDualStats(true);
        }

        while (measure_group_.syncPackage(imu_data_, livox_data_))
        {
            lio_builder_->mapping(measure_group_);
            if (lio_builder_->currentStatus() == fastlio::Status::INITIALIZE)
                continue;
            current_time_ = measure_group_.lidar_time_end;
            current_state_ = lio_builder_->currentState();
            recordTrajectoryPose();
            addKeyPose();
        }

        terminate_flag = true;
        loop_thread_->join();
        spinner.stop();
        bag.close();

        if (total_msgs > 0 && last_progress < 100)
        {
            ROS_INFO("Offline processing progress: 100%% (%lu/%lu)", processed_msgs, total_msgs);
        }
        finalizeAndSave();
        std::cout << "OFFLINE PROCESS DONE!" << std::endl;
    }

    void finalizeAndSave()
    {
        loop_closure_.finalize();
        // Dense TUM poses were recorded with the pre-finalize offset.  Rebase
        // each sample onto the nearest optimized keyframe offset so exported
        // closure reflects the pose-graph (including final-to-start).
        {
            std::lock_guard<std::mutex> lock(shared_data_->mutex);
            const auto &kps = shared_data_->key_poses;
            if (!kps.empty() && !trajectory_.empty())
            {
                size_t k = 0;
                for (Pose6D &pose : trajectory_)
                {
                    while (k + 1 < kps.size() && kps[k + 1].time <= pose.time)
                        ++k;
                    const Pose6D &key = kps[k];
                    const Eigen::Matrix3d offset_rot =
                        key.global_rot * key.local_rot.transpose();
                    const Eigen::Vector3d offset_pos =
                        -key.global_rot * key.local_rot.transpose() * key.local_pos +
                        key.global_pos;
                    pose.addOffset(offset_rot, offset_pos);
                }
                const double key_closure =
                    (kps.back().global_pos - kps.front().global_pos).norm();
                ROS_INFO("Post-PGO keyframe endpoint distance: %.3f m (%zu keyframes)",
                         key_closure, kps.size());
            }
        }
        if (!saveTrajectory(offline_traj_output_))
        {
            ROS_ERROR("Failed to save trajectory to %s", offline_traj_output_.c_str());
        }
        if (offline_save_map_ && !saveFinalMap(offline_map_output_))
        {
            ROS_ERROR("Failed to save map to %s", offline_map_output_.c_str());
        }
        else if (!offline_save_map_)
        {
            ROS_INFO("offline_save_map=false; skipped final PCD generation.");
        }
    }

    bool saveTrajectory(const std::string &path)
    {
        std::ofstream ofs(path);
        if (!ofs.is_open())
        {
            ROS_ERROR("Cannot open trajectory file: %s", path.c_str());
            return false;
        }
        ofs << std::fixed << std::setprecision(6);
        for (const Pose6D &p : trajectory_)
        {
            Eigen::Quaterniond q(p.global_rot);
            ofs << p.time << " "
                << p.global_pos(0) << " " << p.global_pos(1) << " " << p.global_pos(2) << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        }
        ofs.close();
        ROS_INFO("Trajectory saved to %s", path.c_str());
        return true;
    }

    bool saveFinalMap(const std::string &path)
    {
        double used_resolution = save_map_resolution_;
        size_t point_count = 0;
        std::string error;
        if (!writeSparseVoxelMap(path, save_map_resolution_, used_resolution,
                                 point_count, error))
        {
            ROS_ERROR("Final map save failed: %s", error.c_str());
            return false;
        }
        ROS_INFO("Map saved to %s (%zu points, %.3f m sparse voxels)",
                 path.c_str(), point_count, used_resolution);
        return true;
    }

private:
    ros::NodeHandle nh_;
    std::string global_frame_;
    std::string local_frame_;
    std::string body_frame_;
    double current_time_;
    // kf::State current_state_;
    fastlio::state_ikfom current_state_;
    ImuData imu_data_;
    LivoxData livox_data_;
    MeasureGroup measure_group_;
    fastlio::LioParams lio_params_;
    std::shared_ptr<fastlio::LIOBuilder> lio_builder_;
    std::shared_ptr<SharedData> shared_data_;
    std::shared_ptr<ros::Rate> local_rate_;
    double loop_rate_hz_{1.0};
    bool adaptive_rate_{true};
    int lidar_sub_queue_size_{10};
    bool offline_mode_{false};
    bool offline_save_map_{true};
    bool publish_global_map_{false};
    int global_map_keypose_interval_{30};
    double global_map_resolution_{0.25};
    size_t last_global_map_keypose_count_{0};
    uint64_t last_global_map_optimization_revision_{0};
    std::string lidar_type_{"pointcloud2"};
    std::string offline_bag_path_;
    std::string offline_traj_output_;
    std::string offline_map_output_;
    double offline_start_offset_sec_{0.0};
    double offline_duration_sec_{-1.0};
    bool store_keyframe_clouds_{true};
    double keyframe_cloud_resolution_{0.3};
    double save_map_resolution_{0.3};
    int save_map_max_voxels_{3000000};
    int save_map_max_resolution_attempts_{5};
    std::vector<Pose6D> trajectory_;
    LoopClosureThread loop_closure_;
    std::shared_ptr<std::thread> loop_thread_;

    tf2_ros::TransformBroadcaster &br_;

    ros::Subscriber imu_sub_;

    ros::Subscriber livox_sub_;

    ros::Subscriber wheel_odom_sub_;

    ros::Publisher body_cloud_pub_;

    ros::Publisher local_cloud_pub_;

    ros::Publisher global_map_pub_;

    ros::Publisher odom_pub_;

    ros::Publisher loop_mark_pub_;

    ros::Publisher local_path_pub_;

    ros::Publisher global_path_pub_;

    ros::ServiceServer save_map_server_;

    pcl::PCDWriter writer_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_builder_node");
    tf2_ros::TransformBroadcaster br;
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    std::shared_ptr<SharedData> share_date = std::make_shared<SharedData>();
    MapBuilderROS map_builder(br, share_date);
    map_builder.run();
    return 0;
}
