#pragma once

#include "commons.h"
#include "imu_processor.h"
#include "ikd-Tree/ikd_Tree.h"
#include <pcl/filters/voxel_grid.h>

namespace fastlio
{
    enum Status
    {
        INITIALIZE,
        RELOCALIZATION,
        LOCALIZATION,
        MAPPING
    };

    struct LocalMap
    {
        double cube_len;
        double det_range;
        double move_thresh;
        bool is_initialed = false;
        BoxPointType local_map_corner;
        std::vector<BoxPointType> cub_to_rm;
    };
    struct LioParams
    {
        double resolution = 0.1;
        double esikf_min_iteration = 2;
        double esikf_max_iteration = 5;
        double imu_acc_cov = 0.01;
        double imu_gyro_cov = 0.01;
        double imu_acc_bias_cov = 0.0001;
        double imu_gyro_bias_cov = 0.0001;
        int imu_init_samples = 20;

        std::vector<double> imu_ext_rot = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        std::vector<double> imu_ext_pos = {-0.011, -0.02329, 0.04412};

        double cube_len = 1000.0;
        double det_range = 100.0;
        double move_thresh = 1.5;

        // ikd-Tree rebuild controls.  A max of 0 preserves the upstream
        // unlimited asynchronous rebuild behavior.  A positive max bounds
        // the temporary old-tree + new-tree memory peak by selecting smaller
        // subtrees for balance rebuilds.  Deletion-triggered rebuilds remain
        // unlimited unless explicitly capped as well.
        double ikdtree_delete_criterion = 0.5;
        double ikdtree_balance_criterion = 0.6;
        bool ikdtree_async_rebuild_enabled = true;
        int ikdtree_async_rebuild_min_points = Multi_Thread_Rebuild_Point_Num;
        int ikdtree_async_rebuild_max_points = 0;
        bool ikdtree_limit_delete_rebuild_above_max = false;
        bool ikdtree_release_rebuild_storage = true;
        bool ikdtree_print_rebuild_diagnostics = false;
        int ikdtree_diagnostic_interval_frames = 0;

        bool extrinsic_est_en = false;
        bool align_gravity = false;

        // HDL-style tunnel handling for mapping.  When planar scan matching
        // becomes degenerate, LiDAR keeps the strong (lateral) correction and
        // wheel odometry supplies only the weak (longitudinal) velocity.
        bool hdl_directional_constraint = false;
        double hdl_hessian_ratio_enter = 0.05;
        double hdl_hessian_ratio_recover = 0.08;

        // wheel odometry fusion
        bool use_wheel_odom = false;
        // /go2/odom twist.linear is the single wheel-odometry measurement
        // source. Do not also treat its time integral as an independent
        // position observation unless legacy behavior is explicitly needed.
        bool odom_fuse_velocity = true;
        bool odom_fuse_z_velocity = true;
        bool odom_fuse_integrated_position = false;
        bool odom_fuse_angular_velocity = false;
        // Unitree SportModeState.velocity is in the odometry frame. The fusion
        // path below expects body-frame linear velocity.
        bool odom_velocity_in_body_frame = true;
        double odom_max_linear_speed = 3.0;
        double odom_max_angular_speed = 4.0;
        double odom_linear_deadband = 0.01;
        double odom_angular_deadband = 0.02;
        std::string wheel_odom_topic = "/wheel_odom";
        std::vector<double> R_odom_to_imu = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        double odom_pos_noise_std = 0.05;
        double odom_vel_noise_std = 0.1;
        double odom_rot_noise_std = 0.08;
        double odom_rot_innovation_gate = 0.05;
        double odom_rot_covariance_floor_scale = 0.0;
        double odom_covariance_floor_scale = 0.0;
        double odom_z_pos_noise_std = 0.08;
        double odom_z_vel_noise_std = 0.08;
        bool odom_use_orientation = true;
        bool odom_use_absolute_orientation = false;
        bool odom_planar = false;
        bool odom_constrain_z = false;
        bool odom_hard_constrain_z = true;
        bool odom_debug = false;
    };

    class LIOBuilder
    {
    public:
        LIOBuilder(LioParams &params);

        void mapping(const MeasureGroup &meas);

        void sharedUpdateFunc(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

        PointCloudXYZI::Ptr transformToWorld(const PointCloudXYZI::Ptr cloud);

        void trimMap();

        void increaseMap();

        void reset();

        state_ikfom currentState() const { return kf_->get_x(); }
        Status currentStatus() const { return status; }
        std::shared_ptr<KD_TREE<PointType>> getIKDtree() { return ikdtree_; }
        std::shared_ptr<esekfom::esekf<state_ikfom, 12, input_ikfom>> getKF() { return kf_; }
        std::shared_ptr<IMUProcessor> getIMUProcessor() { return imu_processor_; }
        PointCloudXYZI::Ptr cloudWorld() { return cloud_down_world_; }
        PointCloudXYZI::Ptr cloudLidar() { return cloud_down_lidar_; }
        PointCloudXYZI::Ptr cloudUndistortedLidar() { return cloud_undistorted_lidar_; }
        PointCloudXYZI::Ptr cloudUndistortedBody();
        PointCloudXYZI::Ptr cloudDownBody();

        // wheel odom interface
        void pushWheelOdom(const nav_msgs::Odometry::ConstPtr &msg);
        void resetWheelOdom();

    private:
        LioParams params_;
        Status status = Status::INITIALIZE;
        std::shared_ptr<IMUProcessor> imu_processor_;
        std::shared_ptr<esekfom::esekf<state_ikfom, 12, input_ikfom>> kf_;
        std::shared_ptr<KD_TREE<PointType>> ikdtree_;
        pcl::VoxelGrid<PointType> down_size_filter_;

        PointCloudXYZI::Ptr cloud_undistorted_lidar_;
        LocalMap local_map_;

        bool extrinsic_est_en_;
        bool hdl_directional_constraint_ = false;
        double hdl_hessian_ratio_enter_ = 0.05;
        double hdl_hessian_ratio_recover_ = 0.08;
        bool lidar_planar_observability_available_ = false;
        bool lidar_planar_degenerate_ = false;
        double lidar_planar_ratio_ = 1.0;
        Eigen::Vector2d lidar_weak_direction_ = Eigen::Vector2d::UnitX();
        PointCloudXYZI::Ptr cloud_down_lidar_;
        PointCloudXYZI::Ptr cloud_down_world_;
        std::vector<PointVector> nearest_points_;
        std::vector<bool> point_selected_flag_;
        PointCloudXYZI::Ptr norm_vec_;

        PointCloudXYZI::Ptr effect_cloud_lidar_;
        PointCloudXYZI::Ptr effect_norm_vec_;

        // wheel odom fusion
        bool use_wheel_odom_ = false;
        bool odom_fuse_velocity_ = true;
        bool odom_fuse_z_velocity_ = true;
        bool odom_fuse_integrated_position_ = false;
        bool odom_fuse_angular_velocity_ = false;
        bool odom_velocity_in_body_frame_ = true;
        double odom_max_linear_speed_ = 3.0;
        double odom_max_angular_speed_ = 4.0;
        double odom_linear_deadband_ = 0.01;
        double odom_angular_deadband_ = 0.02;
        Eigen::Matrix3d R_odom_to_imu_ = Eigen::Matrix3d::Identity();
        double odom_pos_noise_std_ = 0.05;
        double odom_vel_noise_std_ = 0.1;
        double odom_rot_noise_std_ = 0.08;
        double odom_rot_innovation_gate_ = 0.05;
        double odom_rot_covariance_floor_scale_ = 0.0;
        double odom_covariance_floor_scale_ = 0.0;
        double odom_z_pos_noise_std_ = 0.08;
        double odom_z_vel_noise_std_ = 0.08;
        bool odom_use_orientation_ = true;
        bool odom_use_absolute_orientation_ = false;
        bool odom_planar_ = false;
        bool odom_constrain_z_ = false;
        bool odom_hard_constrain_z_ = true;
        std::deque<WheelOdomData> wheel_odom_buffer_;
        std::mutex wheel_odom_mutex_;
        size_t wheel_odom_max_buf_ = 4000;
        state_ikfom last_state_point_wheel_;
        bool has_last_wheel_state_ = false;
        double last_wheel_odom_frame_time_ = 0.0;
        bool has_last_wheel_time_ = false;
        bool has_wheel_odom_z_ref_ = false;
        double wheel_odom_z_ref_ = 0.0;
        double slam_z_ref_ = 0.0;
        bool has_last_odom_height_ = false;
        bool has_odom_orientation_anchor_ = false;
        Eigen::Matrix3d R_map_odom_anchor_ = Eigen::Matrix3d::Identity();
        double last_odom_height_ = 0.0;
        bool odom_debug_ = false;
        std::uint64_t processed_lidar_frames_ = 0;
        int ikdtree_diagnostic_interval_frames_ = 0;

        bool predictStateWheelOdom(double frame_begin, double frame_end,
                                   const Eigen::Matrix3d &imu_pred_rot,
                                   state_ikfom &state_out,
                                   bool &has_orientation);
        void applyWheelOdomUpdate(const MeasureGroup &meas, const Eigen::Matrix3d &imu_pred_rot);
        void applyDirectionalLidarConstraint(
            const state_ikfom &state_before_lidar,
            const Eigen::Matrix<double, 23, 23> &cov_before_lidar);
        void applyPlanarHeightConstraint();
        void logIKDTreeDiagnostics();
        void configureIKDTree();
    };
}
