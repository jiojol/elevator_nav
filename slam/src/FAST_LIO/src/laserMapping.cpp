// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver2/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic, pcd_save_path;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool   navigation_low_latency = false;
int lidar_type;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

// 里程计相关变量
struct WheelOdomData {
    double stamp = 0.0;
    V3D vel = Zero3d;
    SO3 rot = SO3();  // 里程计的姿态（四元数转换为SO3）
    bool has_pose = false;  // 是否包含姿态信息
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
using WheelOdomBuffer = std::deque<WheelOdomData, Eigen::aligned_allocator<WheelOdomData>>;
WheelOdomBuffer wheel_odom_buffer;
std::mutex mtxWheelOdom;
bool use_wheel_odom = false;
std::string wheel_odom_topic = "/wheel_odom";
M3D Rodom2imu = Eye3d;  // 从里程计坐标系到IMU坐标系的旋转矩阵
double odom_weight = 0.1;  // 里程计融合权重（0.0-1.0，建议0.1-0.2）
double odom_pos_noise_std = 0.05;  // 里程计位置测量噪声标准差（单位：m）
double odom_vel_noise_std = 0.1;   // 里程计速度测量噪声标准差（单位：m/s）
bool enable_lidar_update = true;   // 是否启用lidar测量更新（用于调试滤波融合）
ros::Subscriber subWheelOdom;
double last_wheel_odom_frame_time = 0.0;
bool has_last_wheel_state = false;
bool has_last_wheel_time = false;
state_ikfom last_state_point_wheel;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg) 
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

// 里程计数据回调函数
void wheel_odom_cbk(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    if (!use_wheel_odom) return;
    if (!odomMsg) return;
    
    WheelOdomData data;
    data.stamp = odomMsg->header.stamp.toSec();
    data.vel << odomMsg->twist.twist.linear.x, 
                odomMsg->twist.twist.linear.y,
                odomMsg->twist.twist.linear.z;
    
    // 提取姿态信息（如果可用）
    if (odomMsg->pose.pose.orientation.w != 0.0 || 
        odomMsg->pose.pose.orientation.x != 0.0 ||
        odomMsg->pose.pose.orientation.y != 0.0 ||
        odomMsg->pose.pose.orientation.z != 0.0) {
        Eigen::Quaterniond q_odom(
            odomMsg->pose.pose.orientation.w,
            odomMsg->pose.pose.orientation.x,
            odomMsg->pose.pose.orientation.y,
            odomMsg->pose.pose.orientation.z);
        // 将里程计姿态转换到IMU坐标系
        // 里程计的pose.pose.orientation表示里程计在世界坐标系中的姿态
        // Rodom2imu是从里程计坐标系到IMU坐标系的旋转
        // 如果里程计和IMU的旋转关系是：R_imu = Rodom2imu * R_odom
        // 那么在世界坐标系中：R_imu_world = Rodom2imu * R_odom_world
        M3D R_odom_world = q_odom.toRotationMatrix();
        M3D R_imu_world = Rodom2imu * R_odom_world;
        data.rot = SO3(R_imu_world);
        data.has_pose = true;
    } else {
        data.has_pose = false;
    }

    std::lock_guard<std::mutex> lock(mtxWheelOdom);
    wheel_odom_buffer.push_back(data);
    // 限制缓冲区大小，避免内存溢出
    const size_t max_buffer_size = 4000;
    while (wheel_odom_buffer.size() > max_buffer_size) {
        wheel_odom_buffer.pop_front();
    }
}

// 使用里程计数据预测状态（参考 GeoFlowSlam 的 PredictStateOdom）
bool PredictStateWheelOdom(double frame_begin, double frame_end, state_ikfom &state_out)
{
    if (!use_wheel_odom || !has_last_wheel_state) return false;
    if (frame_end <= frame_begin) return false;
    if (frame_end - frame_begin > 1.0) return false; // 防止时间间隔过大

    std::lock_guard<std::mutex> lock(mtxWheelOdom);
    if (wheel_odom_buffer.empty()) return false;

    // 确定时间范围：从上一帧到当前帧（参考 GeoFlowSlam）
    // 使用 last_wheel_odom_frame_time 作为起始时间，确保速度累积的时间范围与dt匹配
    double odom_start_time = has_last_wheel_time ? last_wheel_odom_frame_time : frame_begin;
    
    // 计算时间间隔：当前帧结束时间减去上一帧时间（参考 GeoFlowSlam）
    double dt = frame_end - odom_start_time;
    if (dt <= 1e-4 || dt > 1.0) {
        ROS_WARN("Wheel odom dt invalid: %.6f (start=%.6f, end=%.6f)", 
                 dt, odom_start_time, frame_end);
        return false;
    }

    // 清理过期的数据（保留时间范围内的数据）
    while (!wheel_odom_buffer.empty() &&
           wheel_odom_buffer.front().stamp < odom_start_time) {
        wheel_odom_buffer.pop_front();
    }

    if (wheel_odom_buffer.empty()) return false;

    // 计算平均速度：分别计算位置更新用的速度和速度输出用的速度
    // 位置更新：使用整个时间段内的平均速度（更准确的位置预测）
    // 速度输出：使用最后几帧的平均速度（更准确的瞬时速度）
    const size_t max_vel_samples = 2;  // 速度输出最多使用最后5帧数据
    
    // 先找到时间范围内的所有数据
    std::vector<V3D> valid_vels;
    std::vector<double> valid_times;
    for (const auto &meas : wheel_odom_buffer) {
        // 只使用从上一帧时间到当前帧结束时间之间的数据
        if (meas.stamp < odom_start_time) continue; // 跳过odom_start_time之前的数据
        if (meas.stamp > frame_end) break;          // 超过frame_end的数据不处理
        valid_vels.push_back(meas.vel);
        valid_times.push_back(meas.stamp);
    }
    
    if (valid_vels.empty()) {
        ROS_DEBUG("No wheel odom data in time range [%.6f, %.6f]", odom_start_time, frame_end);
        return false;
    }
    
    // 1. 计算位置更新用的平均速度（整个时间段内的平均）
    V3D vel_sum_for_pos = Zero3d;
    for (const auto &vel : valid_vels) {
        vel_sum_for_pos += vel;
    }
    V3D avg_vel_odom_for_pos = vel_sum_for_pos / static_cast<double>(valid_vels.size());
    
    // 2. 计算速度输出用的平均速度（最后几帧的平均，更准确的瞬时速度）
    V3D vel_sum_for_vel = Zero3d;
    size_t used_for_vel = 0;
    size_t start_idx = valid_vels.size() > max_vel_samples ? valid_vels.size() - max_vel_samples : 0;
    for (size_t i = start_idx; i < valid_vels.size(); ++i) {
        vel_sum_for_vel += valid_vels[i];
        ++used_for_vel;
    }
    V3D avg_vel_odom_for_vel = vel_sum_for_vel / static_cast<double>(used_for_vel);
    
    // 将里程计速度从里程计坐标系转换到IMU坐标系
    // 注意：这里应该使用Rodom2imu，将里程计坐标系的速度转换到IMU坐标系
    
    // 位置更新用的速度（整个时间段内的平均）
    V3D avg_vel_imu_for_pos = Rodom2imu * avg_vel_odom_for_pos;
    
    // 速度输出用的速度（最后几帧的平均，更准确的瞬时速度）
    V3D avg_vel_imu_for_vel = Rodom2imu * avg_vel_odom_for_vel;
    
    // 计算位移：使用整个时间段内的平均速度（更准确的位置预测）
    // 参考 GeoFlowSlam 的实现
    // GeoFlowSlam: delta_pos = Rodom2camera * avg_vel * dt
    // 我们: delta_body = Rodom2imu * avg_vel_odom_for_pos * dt = avg_vel_imu_for_pos * dt
    // delta_body是IMU body frame中的位移
    V3D delta_body = avg_vel_imu_for_pos * dt;
    
    // 获取上一帧的旋转矩阵（IMU到世界坐标系的旋转）
    // 参考 GeoFlowSlam: last_Rwc 是 camera 到 world 的旋转
    // 我们: last_R_world 是 IMU 到 world 的旋转
    M3D last_R_world = last_state_point_wheel.rot.toRotationMatrix();

    // 更新位置：在上一帧位置基础上加上位移
    // 参考 GeoFlowSlam: cur_twc = last_twc + last_Rwc * dpos_camera
    // 我们: updated_pos = last_pos + last_R_world * delta_body
    // delta_body是IMU body frame中的位移，需要转换到世界坐标系
    V3D updated_pos = last_state_point_wheel.pos + last_R_world * delta_body;
    
    // 更新速度：使用最后几帧的平均速度（更准确的瞬时速度）
    // 参考 GeoFlowSlam: cur_v = mTcb.rotationMatrix().inverse() * avg_vel (转换到camera坐标系)
    // 我们: cur_vel_world = last_R_world * avg_vel_imu_for_vel (转换到世界坐标系)
    // 注意：GeoFlowSlam的速度是在camera坐标系中，而我们的速度是在世界坐标系中
    // 这是因为fast-lio的状态表示方式不同
    V3D cur_vel_world = last_R_world * avg_vel_imu_for_vel;

    // 更新旋转：如果有里程计姿态信息，计算旋转增量
    // 注意：直接使用里程计姿态可能导致不连续和累积误差，应该计算旋转增量
    SO3 updated_rot = last_state_point_wheel.rot; // 默认使用上一帧的旋转
    bool has_odom_rot = false;
    SO3 odom_rot_at_end;
    SO3 odom_rot_at_last;
    
    // 查找时间范围内最新的里程计姿态信息（最接近frame_end的）
    double closest_time = -1.0;
    for (const auto &meas : wheel_odom_buffer) {
        if (meas.stamp <= frame_end && meas.has_pose) {
            if (meas.stamp > closest_time) {
                closest_time = meas.stamp;
                odom_rot_at_end = meas.rot;
                has_odom_rot = true;
            }
        }
    }
    
    // 如果有里程计姿态，计算旋转增量而不是直接使用
    if (has_odom_rot) {
        // 查找上一帧时刻的里程计姿态
        double last_time = has_last_wheel_time ? last_wheel_odom_frame_time : frame_begin;
        double closest_last_time = -1.0;
        bool has_last_odom_rot = false;
        
        for (const auto &meas : wheel_odom_buffer) {
            if (meas.stamp <= last_time && meas.has_pose) {
                if (meas.stamp > closest_last_time) {
                    closest_last_time = meas.stamp;
                    odom_rot_at_last = meas.rot;
                    has_last_odom_rot = true;
                }
            }
        }
        
        if (has_last_odom_rot) {
            // 计算从上一帧EKF旋转到当前里程计旋转的旋转增量
            // 这样可以确保旋转方向正确，并且平滑过渡
            M3D R_last_ekf = last_state_point_wheel.rot.toRotationMatrix();
            M3D R_end = odom_rot_at_end.toRotationMatrix();
            
            // 计算旋转增量：从R_last_ekf到R_end的旋转
            // R_end = R_delta * R_last_ekf，所以 R_delta = R_end * R_last_ekf.transpose()
            M3D R_delta = R_end * R_last_ekf.transpose();
            
            // 检查旋转增量是否合理（避免异常大的旋转）
            // 根据时间间隔动态调整阈值：允许的最大角速度约为 180度/秒
            double dt_rot = closest_time - closest_last_time;
            if (dt_rot < 0.01) dt_rot = 0.01; // 最小时间间隔
            double max_angular_velocity = 180.0 * M_PI / 180.0; // 180度/秒
            double max_rot_delta = max_angular_velocity * dt_rot;
            // 同时设置一个绝对上限：单次旋转增量不超过90度（防止异常数据）
            max_rot_delta = std::min(max_rot_delta, 90.0 * M_PI / 180.0);
            
            V3D rot_vec_delta = Log(R_delta);
            double rot_angle_delta = rot_vec_delta.norm();
            
            if (rot_angle_delta > max_rot_delta) {
                // 旋转增量过大，可能有问题，使用上一帧的旋转
                ROS_WARN("Wheel odom rotation delta too large: %.2f deg (dt=%.3f s, max=%.2f deg), using last frame rotation", 
                         rot_angle_delta * 180.0 / M_PI, dt_rot, max_rot_delta * 180.0 / M_PI);
                updated_rot = last_state_point_wheel.rot;
            } else {
                // 将旋转增量应用到上一帧的EKF状态
                // R_updated = R_delta * R_last_ekf = R_end（因为R_delta = R_end * R_last_ekf.transpose()）
                M3D R_updated = R_delta * R_last_ekf;
                updated_rot = SO3(R_updated);
            }
        } else {
            // 没有找到上一帧的里程计姿态，使用上一帧的旋转（不更新）
            updated_rot = last_state_point_wheel.rot;
        }
    }
    // 如果没有找到姿态信息，使用上一帧的旋转（不更新）

    // 更新状态（融合会在调用函数中完成）
    state_out.pos = updated_pos;
    state_out.vel = cur_vel_world;
    state_out.rot = updated_rot;

    // 更新时间和标志
    last_wheel_odom_frame_time = frame_end;
    has_last_wheel_time = true;

    // 清理已使用的数据（参考 GeoFlowSlam：在预测后立即清空队列）
    // 注意：GeoFlowSlam使用mlQueueOdomData.clear()，我们只清理已使用的数据
    while (!wheel_odom_buffer.empty() && wheel_odom_buffer.front().stamp <= frame_end) 
    {
        wheel_odom_buffer.pop_front();
    }

    return true;
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    // Navigation consumes the newest complete scan rather than replaying a
    // startup/CPU backlog. Keep ALL intervening IMU samples for propagation
    // and deskew; never replace a measurement timestamp with wall time.
    if (navigation_low_latency && !flg_first_scan) {
        while (lidar_buffer.size() > 1) {
            const auto& next_scan = lidar_buffer[1];
            const double duration = next_scan->size() > 1
                ? std::max(lidar_mean_scantime, next_scan->points.back().curvature / 1000.0)
                : lidar_mean_scantime;
            if (last_timestamp_imu < time_buffer[1] + duration) break;
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            lidar_pushed = false;
        }
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();


        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }
        if(lidar_type == MARSIM)
            lidar_end_time = meas.lidar_beg_time;

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to " << pcd_save_path << endl;
            pcd_writer.writeBinary(pcd_save_path, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh.param<bool>("navigation_low_latency", navigation_low_latency, false);
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    nh.param<string>("map_file_path",map_file_path,"");
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<bool>("common/gravity_align_world", p_imu->gravity_align_world, false);
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("cube_side_length",cube_len,200);
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);
    nh.param<double>("mapping/fov_degree",fov_deg,180);
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<string>("pcd_save/path", pcd_save_path, "PCD/scans.pcd");
    if (pcd_save_path.empty()) pcd_save_path = "PCD/scans.pcd";
    if (pcd_save_path.front() != '/') pcd_save_path = root_dir + pcd_save_path;
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    
    // 里程计相关参数
    nh.param<bool>("odom/use_wheel_odom", use_wheel_odom, false);
    nh.param<string>("odom/odom_topic", wheel_odom_topic, "/wheel_odom");
    nh.param<double>("odom/odom_weight", odom_weight, 0.1);  // 里程计融合权重，默认0.1
    nh.param<double>("odom/pos_noise_std", odom_pos_noise_std, 0.05);  // 位置测量噪声标准差，默认0.05m
    nh.param<double>("odom/vel_noise_std", odom_vel_noise_std, 0.1);   // 速度测量噪声标准差，默认0.1 m/s
    nh.param<bool>("odom/enable_lidar_update", enable_lidar_update, true);  // 是否启用lidar测量更新，默认true
    std::vector<double> rod2imu(9, 0.0);
    nh.param<vector<double>>("odom/R_odom_to_imu", rod2imu, vector<double>());
    if (rod2imu.size() == 9) {
        Rodom2imu << rod2imu[0], rod2imu[1], rod2imu[2],
                     rod2imu[3], rod2imu[4], rod2imu[5],
                     rod2imu[6], rod2imu[7], rod2imu[8];
    } else {
        Rodom2imu = Eye3d; // 默认单位矩阵
    }

    p_pre->lidar_type = lidar_type;
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;
    
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu->lidar_type = lidar_type;
    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg, fout_imu_only, fout_odom_only, fout_fused;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    fout_imu_only.open(DEBUG_FILE_DIR("imu_only.txt"),ios::out);      // 纯IMU预测结果
    fout_odom_only.open(DEBUG_FILE_DIR("odom_only.txt"),ios::out);    // 纯里程计预测结果
    fout_fused.open(DEBUG_FILE_DIR("fused.txt"),ios::out);            // 融合后的结果
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;
    
    // 写入文件头
    if (fout_imu_only) {
        fout_imu_only << "# time(s) roll(deg) pitch(deg) yaw(deg) px(m) py(m) pz(m) vx(m/s) vy(m/s) vz(m/s)" << endl;
    }
    if (fout_odom_only) {
        fout_odom_only << "# time(s) roll(deg) pitch(deg) yaw(deg) px(m) py(m) pz(m) vx(m/s) vy(m/s) vz(m/s)" << endl;
    }
    if (fout_fused) {
        fout_fused << "# time(s) roll(deg) pitch(deg) yaw(deg) px(m) py(m) pz(m) vx(m/s) vy(m/s) vz(m/s)" << endl;
    }

    /*** ROS subscribe initialization ***/
    const int lidar_queue_size = navigation_low_latency ? 2 : 200000;
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, lidar_queue_size, livox_pcl_cbk, ros::TransportHints().tcpNoDelay()) : \
        nh.subscribe(lid_topic, lidar_queue_size, standard_pcl_cbk, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    
    // 里程计订阅
    if (use_wheel_odom) {
        subWheelOdom = nh.subscribe<nav_msgs::Odometry>(wheel_odom_topic, 200000, wheel_odom_cbk);
        ROS_INFO("Wheel odometry enabled, topic: %s", wheel_odom_topic.c_str());
    }
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 1);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/Odometry", 100000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 100000);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();
        if(sync_packages(Measures)) 
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            
            // 保存IMU预测的状态（用于文件输出和调试）
            state_ikfom imu_predicted_state = state_point;
            
            // 卡尔曼滤波融合：使用里程计数据来修正IMU预测的状态
            state_ikfom odom_predicted_state;
            bool has_odom_predicted = false;
            
            if (use_wheel_odom && has_last_wheel_state && flg_EKF_inited) {
                // 里程计预测应该基于上一帧激光测量更新后的状态（last_state_point_wheel）
                // 而不是当前IMU预测的状态（state_point）
                odom_predicted_state = last_state_point_wheel; // 从上一帧激光测量更新后的状态开始
                bool wheel_prior_applied = PredictStateWheelOdom(
                    Measures.lidar_beg_time,
                    Measures.lidar_end_time,
                    odom_predicted_state);

                if (wheel_prior_applied) {
                    has_odom_predicted = true;
                    // 卡尔曼滤波更新：使用位置和速度的残差来修正IMU预测的状态
                    // 状态向量结构（23维）：
                    // 0-2: pos, 3-5: rot(SO3), 6-8: offset_R_L_I(SO3), 9-11: offset_T_L_I,
                    // 12-14: vel, 15-17: bg, 18-20: ba, 21-22: grav(S2)
                    
                    // 获取IMU预测的协方差矩阵 (23x23)
                    auto P_imu = kf.get_P();
                    
                    // 定义测量模型：H = [I3 0 ... 0; 0 ... 0 I3 0 ... 0]，测量位置和速度
                    // H: 6x23矩阵，前3行对应位置（索引0-2），后3行对应速度（索引12-14）
                    Eigen::Matrix<double, 6, 23> H = Eigen::Matrix<double, 6, 23>::Zero();
                    // 位置测量：H的前3行，对应状态向量的位置部分（索引0-2）
                    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                    // 速度测量：H的后3行，对应状态向量的速度部分（索引12-14）
                    H.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity();
                    
                    // 定义里程计的测量协方差矩阵 R (6x6: 3个位置 + 3个速度)
                    // 使用从配置文件读取的噪声参数
                    Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();
                    // 位置测量噪声协方差（单位：m^2）
                    R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (odom_pos_noise_std * odom_pos_noise_std);
                    // 速度测量噪声协方差（单位：(m/s)^2）
                    R.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (odom_vel_noise_std * odom_vel_noise_std);
                    
                    // 计算测量残差：z = [位置差; 速度差]
                    Eigen::Matrix<double, 6, 1> residual;
                    residual.head<3>() = odom_predicted_state.pos - state_point.pos;
                    residual.tail<3>() = odom_predicted_state.vel - state_point.vel;
                    
                    // 计算卡尔曼增益：K = P * H^T * (H * P * H^T + R)^(-1)
                    Eigen::Matrix<double, 23, 6> K;
                    Eigen::Matrix<double, 6, 6> S = H * P_imu * H.transpose() + R;
                    K = P_imu * H.transpose() * S.inverse();
                    
                    // 更新状态：x = x + K * residual
                    Eigen::Matrix<double, 23, 1> state_update = K * residual;
                    
                    // 将状态更新应用到state_point
                    // 只更新位置和速度，其他状态保持不变
                    state_point.pos += state_update.segment<3>(0);
                    state_point.vel += state_update.segment<3>(12);
                    // 旋转保持IMU预测的结果，不进行融合
                    // state_point.rot 保持不变（使用IMU预测的旋转）
                    
                    // 更新协方差：P = (I - K * H) * P
                    // 注意：fast-lio的EKF内部会管理协方差，这里我们只更新状态
                    // 如果需要更新协方差，可以在后续的EKF更新中自动处理
                    
                    // 调试信息
/*                     cout<<"================================================"<<endl;
                    cout<<"IMU predicted pos: "<<imu_predicted_state.pos.transpose()<<endl;
                    cout<<"Odom predicted pos: "<<odom_predicted_state.pos.transpose()<<endl;
                    cout<<"Residual pos: "<<residual.head<3>().transpose()<<endl;
                    cout<<"K pos (diag): "<<K.block<3,3>(0,0).diagonal().transpose()<<endl;
                    cout<<"Updated pos: "<<state_point.pos.transpose()<<endl;
                    cout<<"IMU predicted vel: "<<imu_predicted_state.vel.transpose()<<endl;
                    cout<<"Odom predicted vel: "<<odom_predicted_state.vel.transpose()<<endl;
                    cout<<"Residual vel: "<<residual.tail<3>().transpose()<<endl;
                    cout<<"K vel (diag): "<<K.block<3,3>(12,3).diagonal().transpose()<<endl;
                    cout<<"Updated vel: "<<state_point.vel.transpose()<<endl;
 */                    
                    // 更新EKF状态，作为点云匹配的初始猜测
                    kf.change_x(state_point);
                    state_point = kf.get_x();
                }
            }
            
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            
            // 根据配置决定是否执行lidar测量更新
            if (enable_lidar_update) {
                // 执行lidar测量更新（正常的EKF更新）
                kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
                state_point = kf.get_x();
            } else {
                // 跳过lidar测量更新，只使用IMU预测和里程计融合的结果
                // state_point已经在里程计融合后更新过了，这里不需要再次更新
                ROS_DEBUG("Lidar update disabled, using IMU+Odom prediction only");
            }
            
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            // 写入文件：IMU预测、里程计预测、融合后的结果
            double current_time = Measures.lidar_beg_time - first_lidar_time;
            
            // 写入IMU预测结果
            if (fout_imu_only) {
                V3D imu_euler = SO3ToEuler(imu_predicted_state.rot);
                fout_imu_only << setw(20) << current_time 
                             << " " << imu_euler.transpose() 
                             << " " << imu_predicted_state.pos.transpose()
                             << " " << imu_predicted_state.vel.transpose() << endl;
            }
            
            // 写入里程计预测结果（如果有）
            if (fout_odom_only && has_odom_predicted) {
                V3D odom_euler = SO3ToEuler(odom_predicted_state.rot);
                fout_odom_only << setw(20) << current_time 
                              << " " << odom_euler.transpose() 
                              << " " << odom_predicted_state.pos.transpose()
                              << " " << odom_predicted_state.vel.transpose() << endl;
            }
            
            // 写入融合后的结果
            if (fout_fused) {
                fout_fused << setw(20) << current_time 
                          << " " << euler_cur.transpose() 
                          << " " << state_point.pos.transpose()
                          << " " << state_point.vel.transpose() << endl;
            }

            double t_update_end = omp_get_wtime();
            
            // 在EKF更新之后保存状态，用于下一帧的里程计预测
            if (use_wheel_odom && flg_EKF_inited) {
                last_state_point_wheel = state_point;
                has_last_wheel_state = true;
            }

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);
            // Exact-time map localization needs the matching body cloud now,
            // not after incremental-map insertion and world-cloud generation.
            if (navigation_low_latency && scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body);

            /*** add the feature points to map kdtree ***/
            // 如果禁用了lidar更新，也禁用地图更新（因为地图更新依赖于EKF更新后的状态）
            if (enable_lidar_update) {
                t3 = omp_get_wtime();
                map_incremental();
            } else {
                ROS_DEBUG("Map update disabled (lidar update is disabled)");
            }
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (!navigation_low_latency && scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            // publish_effect_world(pubLaserCloudEffect);
            // Publish a current map snapshot instead of letting RViz retain old
            // registered scans after the SLAM pose changes.
            static double last_map_publish_time = -1.0;
            if (pubLaserCloudMap.getNumSubscribers() > 0 &&
                (last_map_publish_time < 0.0 || lidar_end_time - last_map_publish_time >= 1.0))
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->points = ikdtree.PCL_Storage;
                featsFromMap->width = static_cast<uint32_t>(featsFromMap->points.size());
                featsFromMap->height = 1;
                featsFromMap->is_dense = false;
                publish_map(pubLaserCloudMap);
                last_map_publish_time = lidar_end_time;
            }

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to " << pcd_save_path << endl;
        pcd_writer.writeBinary(pcd_save_path, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();
    fout_imu_only.close();
    fout_odom_only.close();
    fout_fused.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
