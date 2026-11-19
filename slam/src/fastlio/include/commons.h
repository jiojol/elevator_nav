#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <queue>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>
#include "IKFoM_toolkit/esekfom/esekfom.hpp"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>

namespace fastlio
{
#define NUM_MATCH_POINTS (5)

#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

#define NUM_MAX_POINTS (10000)
    const double G_m_s2 = 9.81;
    typedef pcl::PointXYZINormal PointType;
    typedef pcl::PointCloud<PointType> PointCloudXYZI;
    typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
    // typedef esekfom::esekf<state_ikfom, 12, input_ikfom> ESEKF;
    struct IMU
    {
        IMU() : acc(Eigen::Vector3d::Zero()), gyro(Eigen::Vector3d::Zero()) {}
        IMU(double t, Eigen::Vector3d a, Eigen::Vector3d g)
            : timestamp(t), acc(a), gyro(g) {}
        IMU(double t, double a1, double a2, double a3, double g1, double g2, double g3)
            : timestamp(t), acc(a1, a2, a3), gyro(g1, g2, g3) {}
        double timestamp;
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
    };

    bool esti_plane(Eigen::Vector4d &out, const PointVector &points, const double &thresh);

    float sq_dist(const PointType &p1, const PointType &p2);

    typedef MTK::vect<3, double> vect3;
    typedef MTK::SO3<double> SO3;
    typedef MTK::S2<double, 98090, 10000, 1> S2;
    typedef MTK::vect<1, double> vect1;
    typedef MTK::vect<2, double> vect2;
    MTK_BUILD_MANIFOLD(state_ikfom,
                       ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I))((vect3, offset_T_L_I))((vect3, vel))((vect3, bg))((vect3, ba))((S2, grav)));

    MTK_BUILD_MANIFOLD(input_ikfom,
                       ((vect3, acc))((vect3, gyro)));

    MTK_BUILD_MANIFOLD(process_noise_ikfom,
                       ((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)));

    MTK::get_cov<process_noise_ikfom>::type process_noise_cov();
    Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in);
    Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in);
    Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in);

    template <typename T, typename Ts>
    Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
    {
        T ang_vel_norm = ang_vel.norm();
        Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

        if (ang_vel_norm > 0.0000001)
        {
            Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
            Eigen::Matrix<T, 3, 3> K;

            K << SKEW_SYM_MATRX(r_axis);

            T r_ang = ang_vel_norm * dt;

            /// Roderigous Tranformation
            return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
        }
        else
        {
            return Eye3;
        }
    }
}
struct ImuData
{
    std::string topic;
    std::mutex mutex;
    std::deque<fastlio::IMU> buffer;
    double last_timestamp = 0;
    void callback(const sensor_msgs::Imu::ConstPtr &msg);
};

struct LivoxData
{
    struct TimestampedCloudSegment
    {
        fastlio::PointCloudXYZI::Ptr cloud;
        double begin = 0.0;
        double end = 0.0;

        TimestampedCloudSegment()
            : cloud(new fastlio::PointCloudXYZI()) {}
    };

    std::string topic;
    std::mutex mutex;
    std::deque<fastlio::PointCloudXYZI::Ptr> buffer;
    std::deque<double> time_buffer;
    double blind = 0.5;
    int filter_num = 3;
    bool use_point_timestamp = false;
    bool normalize_timestamp_segments = false;
    bool sync_dual_lidar_segments = false;
    std::string timestamp_field = "timestamp";
    double timestamp_scale_to_sec = 1.0;
    double timestamp_backward_jump_sec = 0.01;
    int max_timestamp_segments = 4;
    double fallback_scan_period_sec = 0.1;

    // When both use_point_timestamp and sync_dual_lidar_segments are true,
    // /a2/lidar/points is treated as the known dual-JT128 concatenated format.
    // The first monotonic timestamp run is the front scan and the second is the
    // rear scan.  The rear scan may arrive one message late, so retain only a
    // few bounded scans and match them by their absolute FLOAT64 timestamps.
    double dual_sync_tolerance_sec = 0.002;
    double dual_hold_sec = 0.2;
    std::size_t dual_max_pending = 4;
    std::size_t max_output_buffer_frames = 32;
    double dual_min_segment_duration_sec = 0.08;
    double dual_max_segment_duration_sec = 0.12;
    double dual_idle_flush_wall_sec = 1.0;
    bool dual_enable_idle_flush = true;

    std::deque<TimestampedCloudSegment> dual_front_pending;
    std::deque<TimestampedCloudSegment> dual_rear_pending;
    double dual_newest_front_time = 0.0;
    double dual_last_emitted_time = 0.0;
    bool dual_has_newest_front = false;
    bool dual_has_last_emitted = false;
    ros::WallTime dual_last_input_wall;
    ros::WallTimer dual_flush_timer;
    bool dual_flush_timer_started = false;

    std::uint64_t dual_input_frames = 0;
    std::uint64_t dual_valid_frames = 0;
    std::uint64_t dual_paired_frames = 0;
    std::uint64_t dual_front_only_frames = 0;
    std::uint64_t dual_duplicate_rear_dropped = 0;
    std::uint64_t dual_stale_rear_dropped = 0;
    std::uint64_t dual_output_backlog_dropped = 0;
    std::uint64_t dual_invalid_frames = 0;
    std::size_t dual_max_front_pending_seen = 0;
    std::size_t dual_max_rear_pending_seen = 0;

    double last_timestamp = 0;
    // Same meaning as FAST-LIO common/time_offset_lidar_to_imu:
    // lidar stamp used for sync is header.stamp - time_offset_lidar_to_imu.
    double time_offset_lidar_to_imu = 0.0;
    void callback(const livox_ros_driver2::CustomMsg::ConstPtr &msg);
    void callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void livox2pcl(const livox_ros_driver2::CustomMsg::ConstPtr &msg, fastlio::PointCloudXYZI::Ptr &out);
    void pointCloud2pcl(const sensor_msgs::PointCloud2::ConstPtr &msg, fastlio::PointCloudXYZI::Ptr &out);
    bool pointCloud2DualSegments(const sensor_msgs::PointCloud2::ConstPtr &msg,
                                 TimestampedCloudSegment &front,
                                 TimestampedCloudSegment &rear);
    void processDualPending(bool force_all = false);
    void emitDualScan(const TimestampedCloudSegment &front,
                      const TimestampedCloudSegment *rear);
    void clearDualPending();
    void logDualStats(bool force = false) const;
    void dualFlushTimerCallback(const ros::WallTimerEvent &event);
};

struct MeasureGroup
{
    double lidar_time_begin = 0.0;
    double lidar_time_end = 0.0;
    bool lidar_pushed = false;
    fastlio::PointCloudXYZI::Ptr lidar;
    std::deque<fastlio::IMU> imus;
    bool syncPackage(ImuData &imu_data, LivoxData &livox_data);
};

struct WheelOdomData
{
    double stamp = 0.0;
    Eigen::Vector3d pos = Eigen::Vector3d::Zero(); // 里程计位置（odom系）
    Eigen::Vector3d vel = Eigen::Vector3d::Zero(); // 里程计线速度（机体系）
    Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero(); // 里程计角速度（机体系）
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity(); // 里程计姿态（机体系->odom系），若无姿态则保持单位阵
    bool has_pose = false;
};

nav_msgs::Odometry eigen2Odometry(const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos, const std::string &frame_id, const std::string &child_frame_id, const double &timestamp);

geometry_msgs::TransformStamped eigen2Transform(const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos, const std::string &frame_id, const std::string &child_frame_id, const double &timestamp);

sensor_msgs::PointCloud2 pcl2msg(fastlio::PointCloudXYZI::Ptr inp, std::string &frame_id, const double &timestamp);

Eigen::Vector3d rotate2rpy(Eigen::Matrix3d &rot);
