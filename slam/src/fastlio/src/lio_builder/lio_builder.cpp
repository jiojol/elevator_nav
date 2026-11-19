#include "lio_builder/lio_builder.h"
#include "localizer/tunnel_fusion.h"

// #include <chrono>
#include <sstream>
namespace fastlio
{
    /**
     * @brief
     */
    LIOBuilder::LIOBuilder(LioParams &params) : params_(params)
    { // 初始化ESIKF
        kf_ = std::make_shared<esekfom::esekf<state_ikfom, 12, input_ikfom>>();
        std::vector<double> epsi(23, 0.001);
        kf_->init_dyn_share(
            get_f, df_dx, df_dw,
            [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
            { sharedUpdateFunc(s, ekfom_data); },
            params.esikf_max_iteration, epsi.data());

        // 检查并修正外参参数尺寸，避免参数读取错误导致越界
        if (params.imu_ext_rot.size() != 9)
        {
            ROS_WARN_STREAM("lio_builder/imu_ext_rot size = " << params.imu_ext_rot.size()
                            << ", expected 9. Reset to identity.");
            params.imu_ext_rot = {1, 0, 0,
                                  0, 1, 0,
                                  0, 0, 1};
        }
        if (params.imu_ext_pos.size() != 3)
        {
            ROS_WARN_STREAM("lio_builder/imu_ext_pos size = " << params.imu_ext_pos.size()
                            << ", expected 3. Reset to zero.");
            params.imu_ext_pos = {0.0, 0.0, 0.0};
        }
        if (params.R_odom_to_imu.size() != 9)
        {
            ROS_WARN_STREAM("odom/R_odom_to_imu size = " << params.R_odom_to_imu.size()
                            << ", expected 9. Reset to identity.");
            params.R_odom_to_imu = {1, 0, 0,
                                    0, 1, 0,
                                    0, 0, 1};
        }

        // 初始化IMUProcessor
        imu_processor_ = std::make_shared<IMUProcessor>(kf_);
        imu_processor_->setCov(params.imu_gyro_cov, params.imu_acc_cov, params.imu_gyro_bias_cov, params.imu_acc_bias_cov);
        imu_processor_->setMaxInitCount(params.imu_init_samples);
        Eigen::Matrix3d rot_ext;
        Eigen::Vector3d pos_ext;
        rot_ext << params.imu_ext_rot[0], params.imu_ext_rot[1], params.imu_ext_rot[2],
            params.imu_ext_rot[3], params.imu_ext_rot[4], params.imu_ext_rot[5],
            params.imu_ext_rot[6], params.imu_ext_rot[7], params.imu_ext_rot[8];
        pos_ext << params.imu_ext_pos[0], params.imu_ext_pos[1], params.imu_ext_pos[2];
        imu_processor_->setExtParams(rot_ext, pos_ext);
        imu_processor_->setAlignGravity(params.align_gravity);
        hdl_directional_constraint_ = params.hdl_directional_constraint;
        hdl_hessian_ratio_enter_ = std::max(0.0, params.hdl_hessian_ratio_enter);
        hdl_hessian_ratio_recover_ =
            std::max(hdl_hessian_ratio_enter_, params.hdl_hessian_ratio_recover);
        use_wheel_odom_ = params.use_wheel_odom;
        odom_fuse_velocity_ = params.odom_fuse_velocity;
        odom_fuse_z_velocity_ = params.odom_fuse_z_velocity;
        odom_fuse_integrated_position_ = params.odom_fuse_integrated_position;
        odom_fuse_angular_velocity_ = params.odom_fuse_angular_velocity;
        odom_velocity_in_body_frame_ = params.odom_velocity_in_body_frame;
        odom_max_linear_speed_ = std::max(0.0, params.odom_max_linear_speed);
        odom_max_angular_speed_ = std::max(0.0, params.odom_max_angular_speed);
        odom_linear_deadband_ = std::max(0.0, params.odom_linear_deadband);
        odom_angular_deadband_ = std::max(0.0, params.odom_angular_deadband);
        if (use_wheel_odom_ && !odom_fuse_velocity_ &&
            !odom_fuse_integrated_position_ &&
            !odom_fuse_angular_velocity_ && !params.odom_use_orientation)
        {
            ROS_WARN("wheel odometry is enabled but all odometry measurement blocks are disabled");
        }
        if (use_wheel_odom_ && odom_fuse_velocity_ &&
            odom_fuse_integrated_position_)
        {
            ROS_WARN("wheel twist is fused as both velocity and integrated position; "
                     "these observations are correlated (legacy mode)");
        }
        R_odom_to_imu_ << params.R_odom_to_imu[0], params.R_odom_to_imu[1], params.R_odom_to_imu[2],
            params.R_odom_to_imu[3], params.R_odom_to_imu[4], params.R_odom_to_imu[5],
            params.R_odom_to_imu[6], params.R_odom_to_imu[7], params.R_odom_to_imu[8];
        odom_pos_noise_std_ = params.odom_pos_noise_std;
        odom_vel_noise_std_ = params.odom_vel_noise_std;
        odom_rot_noise_std_ = params.odom_rot_noise_std;
        odom_rot_innovation_gate_ = params.odom_rot_innovation_gate;
        odom_rot_covariance_floor_scale_ =
            std::max(0.0, params.odom_rot_covariance_floor_scale);
        odom_covariance_floor_scale_ = std::max(0.0, params.odom_covariance_floor_scale);
        odom_z_pos_noise_std_ = params.odom_z_pos_noise_std;
        odom_z_vel_noise_std_ = params.odom_z_vel_noise_std;
        odom_use_orientation_ = params.odom_use_orientation;
        odom_use_absolute_orientation_ =
            params.odom_use_absolute_orientation;
        odom_planar_ = params.odom_planar;
        odom_constrain_z_ = params.odom_constrain_z;
        odom_hard_constrain_z_ = params.odom_hard_constrain_z;
        odom_debug_ = params.odom_debug;
        // 初始化KDTree
        ikdtree_ = std::make_shared<KD_TREE<PointType>>();
        configureIKDTree();
        // 初始化点云采样器
        down_size_filter_.setLeafSize(params.resolution, params.resolution, params.resolution);

        // 初始化local_map
        local_map_.cube_len = params.cube_len;
        local_map_.det_range = params.det_range;
        local_map_.move_thresh = params.move_thresh;

        extrinsic_est_en_ = params.extrinsic_est_en;

        cloud_down_lidar_.reset(new PointCloudXYZI);

        cloud_down_world_.reset(new PointCloudXYZI(NUM_MAX_POINTS, 1));
        norm_vec_.reset(new PointCloudXYZI(NUM_MAX_POINTS, 1));

        effect_cloud_lidar_.reset(new PointCloudXYZI(NUM_MAX_POINTS, 1));
        effect_norm_vec_.reset(new PointCloudXYZI(NUM_MAX_POINTS, 1));

        nearest_points_.resize(NUM_MAX_POINTS);
        point_selected_flag_.resize(NUM_MAX_POINTS, false);
    }

    void LIOBuilder::configureIKDTree()
    {
        ikdtree_->set_downsample_param(params_.resolution);
        ikdtree_->Set_delete_criterion_param(
            static_cast<float>(std::max(0.05, std::min(0.95,
                                                      params_.ikdtree_delete_criterion))));
        ikdtree_->Set_balance_criterion_param(
            static_cast<float>(std::max(0.5, std::min(0.99,
                                                     params_.ikdtree_balance_criterion))));
        ikdtree_->ConfigureRebuild(
            params_.ikdtree_async_rebuild_enabled,
            params_.ikdtree_async_rebuild_min_points,
            params_.ikdtree_async_rebuild_max_points,
            params_.ikdtree_limit_delete_rebuild_above_max,
            params_.ikdtree_release_rebuild_storage,
            params_.ikdtree_print_rebuild_diagnostics);
        ikdtree_diagnostic_interval_frames_ =
            std::max(0, params_.ikdtree_diagnostic_interval_frames);
    }

    /**
     * @brief 将点云转换到世界坐标系下
     * @param cloud: lidar系下的点云
     *
     */
    PointCloudXYZI::Ptr LIOBuilder::transformToWorld(const PointCloudXYZI::Ptr cloud)
    {
        PointCloudXYZI::Ptr cloud_world(new PointCloudXYZI);
        Eigen::Matrix3d rot = kf_->get_x().rot.toRotationMatrix();
        Eigen::Vector3d pos = kf_->get_x().pos;
        Eigen::Matrix3d rot_ext = kf_->get_x().offset_R_L_I.toRotationMatrix();
        Eigen::Vector3d pos_ext = kf_->get_x().offset_T_L_I;
        cloud_world->reserve(cloud->size());
        for (auto &p : cloud->points)
        {
            Eigen::Vector3d point(p.x, p.y, p.z);
            point = rot * (rot_ext * point + pos_ext) + pos;
            PointType p_world;
            p_world.x = point(0);
            p_world.y = point(1);
            p_world.z = point(2);
            p_world.intensity = p.intensity;
            cloud_world->points.push_back(p_world);
        }
        return cloud_world;
    }

    /**
     * @brief
     */
    void LIOBuilder::mapping(const MeasureGroup &meas)
    {
        if (!imu_processor_->operator()(meas, cloud_undistorted_lidar_))
            return;

        down_size_filter_.setInputCloud(cloud_undistorted_lidar_);
        down_size_filter_.filter(*cloud_down_lidar_);
        ++processed_lidar_frames_;

        // Dual-LiDAR frames can retain more than the historical fixed
        // NUM_MAX_POINTS=10000 after voxel filtering.  Resize every indexed
        // workspace before it is accessed; otherwise the update and map
        // insertion loops write past the end of their buffers.
        const size_t downsampled_size = cloud_down_lidar_->size();
        // These clouds are also published/returned, so keep their logical
        // size equal to the current frame instead of exposing stale tail
        // points left by a previously larger frame.
        cloud_down_world_->resize(downsampled_size);
        norm_vec_->resize(downsampled_size);
        if (effect_cloud_lidar_->size() < downsampled_size)
            effect_cloud_lidar_->resize(downsampled_size);
        if (effect_norm_vec_->size() < downsampled_size)
            effect_norm_vec_->resize(downsampled_size);
        if (nearest_points_.size() < downsampled_size)
            nearest_points_.resize(downsampled_size);
        if (point_selected_flag_.size() < downsampled_size)
            point_selected_flag_.resize(downsampled_size, false);

        if (status == Status::INITIALIZE)
        {
            // 初始化ikd_tree
            PointCloudXYZI::Ptr point_world = transformToWorld(cloud_down_lidar_);
            ikdtree_->Build(point_world->points);
            status = Status::MAPPING;
            logIKDTreeDiagnostics();
            return;
        }

        trimMap();
        const Eigen::Matrix3d imu_pred_rot = kf_->get_x().rot.toRotationMatrix();
        const state_ikfom state_before_lidar = kf_->get_x();
        const Eigen::Matrix<double, 23, 23> cov_before_lidar = kf_->get_P();
        lidar_planar_observability_available_ = false;
        // auto tic = std::chrono::system_clock::now();
        double solve_H_time = 0;
        kf_->update_iterated_dyn_share_modified(0.001, solve_H_time);
        applyDirectionalLidarConstraint(state_before_lidar, cov_before_lidar);
        // Keep wheel odometry as the posterior constraint. In a repetitive
        // tunnel, the LiDAR update can otherwise overwrite the motion prior.
        applyWheelOdomUpdate(meas, imu_pred_rot);
        applyPlanarHeightConstraint();
        // auto toc = std::chrono::system_clock::now();
        // std::chrono::duration<double> duration = toc - tic;
        // std::cout << duration.count() * 1000 << std::endl;
        increaseMap();
        logIKDTreeDiagnostics();

        if (use_wheel_odom_)
        {
            last_state_point_wheel_ = kf_->get_x();
            has_last_wheel_state_ = true;
            last_wheel_odom_frame_time_ = meas.lidar_time_end;
            has_last_wheel_time_ = true;
        }
    }

    void LIOBuilder::logIKDTreeDiagnostics()
    {
        if (ikdtree_diagnostic_interval_frames_ <= 0 ||
            processed_lidar_frames_ %
                    static_cast<std::uint64_t>(ikdtree_diagnostic_interval_frames_) !=
                0)
            return;

        const int tree_size = ikdtree_->size();
        const int valid_points = ikdtree_->validnum();
        float alpha_balance = 0.0f;
        float alpha_delete = 0.0f;
        if (tree_size > 0)
            ikdtree_->root_alpha(alpha_balance, alpha_delete);
        const auto rebuild = ikdtree_->rebuild_diagnostics();
        ROS_INFO("IKD stats: frame=%llu size=%d valid=%d alpha_bal=%.4f "
                 "alpha_del=%.4f rebuild_running=%d target=%d storage=%d "
                 "logger=%d logger_peak=%d completed=%llu "
                 "skipped_balance=%llu skipped_delete=%llu",
                 static_cast<unsigned long long>(processed_lidar_frames_),
                 tree_size, valid_points, alpha_balance, alpha_delete,
                 rebuild.running ? 1 : 0, rebuild.target_points,
                 rebuild.storage_points, rebuild.logger_size,
                 rebuild.max_logger_size,
                 static_cast<unsigned long long>(rebuild.completed),
                 static_cast<unsigned long long>(rebuild.skipped_balance),
                 static_cast<unsigned long long>(rebuild.skipped_delete));
    }

    /**
     * @brief 根据局部地图的范围修建ikdtree中点云
     */
    void LIOBuilder::trimMap()
    {
        local_map_.cub_to_rm.clear();
        state_ikfom state = kf_->get_x();
        Eigen::Vector3d pos_lidar = state.pos + state.rot.toRotationMatrix() * state.offset_T_L_I;
        // 根据lidar的位置进行局部地图的初始化
        if (!local_map_.is_initialed)
        {
            for (int i = 0; i < 3; i++)
            {
                local_map_.local_map_corner.vertex_min[i] = pos_lidar[i] - local_map_.cube_len / 2.0;
                local_map_.local_map_corner.vertex_max[i] = pos_lidar[i] + local_map_.cube_len / 2.0;
            }
            local_map_.is_initialed = true;
            return;
        }

        float dist_to_map_edge[3][2];
        bool need_move = false;
        double det_thresh = local_map_.move_thresh * local_map_.det_range;
        // 如果靠近地图边缘 则需要进行地图的移动
        for (int i = 0; i < 3; i++)
        {
            dist_to_map_edge[i][0] = fabs(pos_lidar(i) - local_map_.local_map_corner.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_lidar(i) - local_map_.local_map_corner.vertex_max[i]);

            if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh)
                need_move = true;
        }
        if (!need_move)
            return;

        BoxPointType new_corner, temp_corner;
        new_corner = local_map_.local_map_corner;
        float mov_dist = std::max((local_map_.cube_len - 2.0 * local_map_.move_thresh * local_map_.det_range) * 0.5 * 0.9,
                                  double(local_map_.det_range * (local_map_.move_thresh - 1)));
        // 更新局部地图
        for (int i = 0; i < 3; i++)
        {
            temp_corner = local_map_.local_map_corner;
            if (dist_to_map_edge[i][0] <= det_thresh)
            {
                new_corner.vertex_max[i] -= mov_dist;
                new_corner.vertex_min[i] -= mov_dist;
                temp_corner.vertex_min[i] = local_map_.local_map_corner.vertex_max[i] - mov_dist;
                local_map_.cub_to_rm.push_back(temp_corner);
            }
            else if (dist_to_map_edge[i][1] <= det_thresh)
            {
                new_corner.vertex_max[i] += mov_dist;
                new_corner.vertex_min[i] += mov_dist;
                temp_corner.vertex_max[i] = local_map_.local_map_corner.vertex_min[i] + mov_dist;
                local_map_.cub_to_rm.push_back(temp_corner);
            }
        }
        local_map_.local_map_corner = new_corner;
        // 强制删除历史点云
        PointVector points_history;
        ikdtree_->acquire_removed_points(points_history);

        // 删除局部地图之外的点云
        if (local_map_.cub_to_rm.size() > 0)
            ikdtree_->Delete_Point_Boxes(local_map_.cub_to_rm);
        return;
    }

    /**
     * @brief
     */
    void LIOBuilder::increaseMap()
    {
        if (status == Status::INITIALIZE)
            return;
        if (cloud_down_lidar_->empty())
            return;

        int size = cloud_down_lidar_->size();

        PointVector point_to_add;
        PointVector point_no_need_downsample;

        point_to_add.reserve(size);
        point_no_need_downsample.reserve(size);

        state_ikfom state = kf_->get_x();
        for (int i = 0; i < size; i++)
        {
            const PointType &p = cloud_down_lidar_->points[i];
            Eigen::Vector3d point(p.x, p.y, p.z);
            point = state.rot.toRotationMatrix() * (state.offset_R_L_I.toRotationMatrix() * point + state.offset_T_L_I) + state.pos;
            cloud_down_world_->points[i].x = point(0);
            cloud_down_world_->points[i].y = point(1);
            cloud_down_world_->points[i].z = point(2);
            cloud_down_world_->points[i].intensity = cloud_down_lidar_->points[i].intensity;
            // 如果该点附近没有近邻点则需要添加到地图中
            if (nearest_points_[i].empty())
            {
                point_to_add.push_back(cloud_down_world_->points[i]);
                continue;
            }

            const PointVector &points_near = nearest_points_[i];
            bool need_add = true;
            PointType downsample_result, mid_point;
            mid_point.x = std::floor(cloud_down_world_->points[i].x / params_.resolution) * params_.resolution + 0.5 * params_.resolution;
            mid_point.y = std::floor(cloud_down_world_->points[i].y / params_.resolution) * params_.resolution + 0.5 * params_.resolution;
            mid_point.z = std::floor(cloud_down_world_->points[i].z / params_.resolution) * params_.resolution + 0.5 * params_.resolution;

            // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * params_.resolution && fabs(points_near[0].y - mid_point.y) > 0.5 * params_.resolution && fabs(points_near[0].z - mid_point.z) > 0.5 * params_.resolution)
            {
                point_no_need_downsample.push_back(cloud_down_world_->points[i]);
                continue;
            }
            float dist = sq_dist(cloud_down_world_->points[i], mid_point);

            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                // 如果该点的近邻点较少，则需要加入到地图中
                if (points_near.size() < NUM_MATCH_POINTS)
                    break;
                // 如果该点的近邻点距离voxel中心点的距离比该点距离voxel中心点更近，则不需要加入该点
                if (sq_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                point_to_add.push_back(cloud_down_world_->points[i]);
        }
        int add_point_size = ikdtree_->Add_Points(point_to_add, true);
        ikdtree_->Add_Points(point_no_need_downsample, false);
    }

    /**
     * @brief
     */
    void LIOBuilder::sharedUpdateFunc(state_ikfom &state, esekfom::dyn_share_datastruct<double> &share_data)
    {
        int size = cloud_down_lidar_->size();
#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for

#endif
        for (int i = 0; i < size; i++)
        {
            PointType &point_body = cloud_down_lidar_->points[i];
            PointType &point_world = cloud_down_world_->points[i];
            Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
            Eigen::Vector3d point_world_vec = state.rot.toRotationMatrix() * (state.offset_R_L_I.toRotationMatrix() * point_body_vec + state.offset_T_L_I) + state.pos;
            point_world.x = point_world_vec(0);
            point_world.y = point_world_vec(1);
            point_world.z = point_world_vec(2);
            point_world.intensity = point_body.intensity;

            std::vector<float> point_sq_dist(NUM_MATCH_POINTS);
            auto &points_near = nearest_points_[i];
            if (share_data.converge)
            {
                ikdtree_->Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, point_sq_dist);
                if (points_near.size() >= NUM_MATCH_POINTS && point_sq_dist[NUM_MATCH_POINTS - 1] <= 5)
                    point_selected_flag_[i] = true;
                else
                    point_selected_flag_[i] = false;
            }
            if (!point_selected_flag_[i])
                continue;

            Eigen::Vector4d pabcd;
            point_selected_flag_[i] = false;

            // 估计平面法向量，同时计算点面距离，计算的值存入intensity
            if (esti_plane(pabcd, points_near, 0.1))
            {
                double pd2 = pabcd(0) * point_world_vec(0) + pabcd(1) * point_world_vec(1) + pabcd(2) * point_world_vec(2) + pabcd(3);
                // 和点面距离正相关，和点的远近距离负相关
                double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
                if (s > 0.9)
                {
                    point_selected_flag_[i] = true;
                    norm_vec_->points[i].x = pabcd(0);
                    norm_vec_->points[i].y = pabcd(1);
                    norm_vec_->points[i].z = pabcd(2);
                    norm_vec_->points[i].intensity = pd2;
                }
            }
        }

        int effect_feat_num = 0;
        for (int i = 0; i < size; i++)
        {
            if (!point_selected_flag_[i])
                continue;
            effect_cloud_lidar_->points[effect_feat_num] = cloud_down_lidar_->points[i];
            effect_norm_vec_->points[effect_feat_num] = norm_vec_->points[i];
            effect_feat_num++;
        }
        if (effect_feat_num < 1)
        {
            share_data.valid = false;
            ROS_INFO("NO Effective Points!");
            return;
        }

        share_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num, 12);
        share_data.h.resize(effect_feat_num);

        for (int i = 0; i < effect_feat_num; i++)
        {
            const PointType &laser_p = effect_cloud_lidar_->points[i];
            const PointType &norm_p = effect_norm_vec_->points[i];
            Eigen::Vector3d laser_p_vec(laser_p.x, laser_p.y, laser_p.z);
            Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
            Eigen::Vector3d temp_vec = state.offset_R_L_I.toRotationMatrix() * laser_p_vec + state.offset_T_L_I;
            Eigen::Matrix3d temp_mat;
            temp_mat << SKEW_SYM_MATRX(temp_vec);
            Eigen::Matrix<double, 1, 3> B = -norm_vec.transpose() * state.rot.toRotationMatrix() * temp_mat;
            share_data.h_x.block<1, 3>(i, 0) = norm_vec.transpose();
            share_data.h_x.block<1, 3>(i, 3) = B;
            if (extrinsic_est_en_)
            {
                temp_mat << SKEW_SYM_MATRX(laser_p_vec);
                Eigen::Matrix<double, 1, 3> C = -norm_vec.transpose() * state.rot.toRotationMatrix() * state.offset_R_L_I.toRotationMatrix() * temp_mat;
                Eigen::Matrix<double, 1, 3> D = norm_vec.transpose() * state.rot.toRotationMatrix();
                share_data.h_x.block<1, 3>(i, 6) = C;
                share_data.h_x.block<1, 3>(i, 9) = D;
            }
            share_data.h(i) = -norm_p.intensity;
        }

        if (hdl_directional_constraint_)
        {
            Matrix6d hessian = Matrix6d::Zero();
            hessian = share_data.h_x.leftCols(6).transpose() *
                      share_data.h_x.leftCols(6);
            const TranslationObservability observability =
                planarTranslationObservability(hessian);
            lidar_planar_observability_available_ = observability.available;
            if (observability.available)
            {
                lidar_planar_ratio_ = observability.ratio;
                lidar_weak_direction_ = observability.weak_direction;
            }
        }
    }

    void LIOBuilder::applyDirectionalLidarConstraint(
        const state_ikfom &state_before_lidar,
        const Eigen::Matrix<double, 23, 23> &cov_before_lidar)
    {
        if (!hdl_directional_constraint_ ||
            !lidar_planar_observability_available_)
            return;

        const bool was_degenerate = lidar_planar_degenerate_;
        lidar_planar_degenerate_ = updateDegeneracyLatch(
            lidar_planar_degenerate_, lidar_planar_ratio_,
            hdl_hessian_ratio_enter_, hdl_hessian_ratio_recover_);
        if (was_degenerate != lidar_planar_degenerate_)
        {
            ROS_WARN_STREAM("HDL tunnel directional constraint "
                            << (lidar_planar_degenerate_ ? "enabled" : "released")
                            << ", planar Hessian ratio=" << lidar_planar_ratio_);
        }
        if (!lidar_planar_degenerate_)
            return;

        Eigen::Vector2d weak = lidar_weak_direction_;
        if (!weak.allFinite() || weak.norm() < 1e-9)
            return;
        weak.normalize();

        // Remove the weak/longitudinal part of this frame's LiDAR position
        // correction.  Rotation, height and the strong/lateral correction are
        // still supplied by scan matching.
        state_ikfom state_after_lidar = kf_->get_x();
        Eigen::Vector2d correction(
            state_after_lidar.pos.x() - state_before_lidar.pos.x(),
            state_after_lidar.pos.y() - state_before_lidar.pos.y());
        correction -= weak * weak.dot(correction);
        state_after_lidar.pos.x() = state_before_lidar.pos.x() + correction.x();
        state_after_lidar.pos.y() = state_before_lidar.pos.y() + correction.y();

        // The scan Jacobian has no direct velocity rows, but ESKF
        // cross-covariance can still inject a weak-axis velocity correction.
        // Remove it as well so that the following wheel update is the sole
        // longitudinal velocity observation.
        Eigen::Vector2d velocity_correction(
            state_after_lidar.vel.x() - state_before_lidar.vel.x(),
            state_after_lidar.vel.y() - state_before_lidar.vel.y());
        velocity_correction -= weak * weak.dot(velocity_correction);
        state_after_lidar.vel.x() =
            state_before_lidar.vel.x() + velocity_correction.x();
        state_after_lidar.vel.y() =
            state_before_lidar.vel.y() + velocity_correction.y();
        kf_->change_x(state_after_lidar);

        // LiDAR must not make the weak position direction artificially
        // certain. Preserve its pre-scan variance while retaining the
        // posterior covariance in all orthogonal directions.
        Eigen::Matrix<double, 23, 1> weak_state =
            Eigen::Matrix<double, 23, 1>::Zero();
        weak_state(0) = weak.x();
        weak_state(1) = weak.y();
        Eigen::Matrix<double, 23, 1> weak_velocity_state =
            Eigen::Matrix<double, 23, 1>::Zero();
        weak_velocity_state(12) = weak.x();
        weak_velocity_state(13) = weak.y();
        const Eigen::Matrix<double, 23, 23> weak_projector =
            weak_state * weak_state.transpose() +
            weak_velocity_state * weak_velocity_state.transpose();
        const Eigen::Matrix<double, 23, 23> strong_projector =
            Eigen::Matrix<double, 23, 23>::Identity() - weak_projector;
        const Eigen::Matrix<double, 23, 23> cov_after_lidar = kf_->get_P();
        Eigen::Matrix<double, 23, 23> filtered_cov =
            strong_projector * cov_after_lidar * strong_projector.transpose() +
            weak_projector * cov_before_lidar * weak_projector.transpose();
        filtered_cov = 0.5 * (filtered_cov + filtered_cov.transpose());
        kf_->change_P(filtered_cov);
    }

    PointCloudXYZI::Ptr LIOBuilder::cloudUndistortedBody()
    {
        PointCloudXYZI::Ptr cloud_undistorted_body(new PointCloudXYZI);
        Eigen::Matrix3d rot = kf_->get_x().offset_R_L_I.toRotationMatrix();
        Eigen::Vector3d pos = kf_->get_x().offset_T_L_I;
        cloud_undistorted_body->reserve(cloud_undistorted_lidar_->size());
        for (auto &p : cloud_undistorted_lidar_->points)
        {
            Eigen::Vector3d point(p.x, p.y, p.z);
            point = rot * point + pos;
            PointType p_body;
            p_body.x = point(0);
            p_body.y = point(1);
            p_body.z = point(2);
            p_body.intensity = p.intensity;
            cloud_undistorted_body->points.push_back(p_body);
        }
        return cloud_undistorted_body;
    }

    PointCloudXYZI::Ptr LIOBuilder::cloudDownBody()
    {
        PointCloudXYZI::Ptr cloud_down_body(new PointCloudXYZI);
        Eigen::Matrix3d rot = kf_->get_x().offset_R_L_I.toRotationMatrix();
        Eigen::Vector3d pos = kf_->get_x().offset_T_L_I;
        cloud_down_body->reserve(cloud_down_lidar_->size());
        for (auto &p : cloud_down_lidar_->points)
        {
            Eigen::Vector3d point(p.x, p.y, p.z);
            point = rot * point + pos;
            PointType p_body;
            p_body.x = point(0);
            p_body.y = point(1);
            p_body.z = point(2);
            p_body.intensity = p.intensity;
            cloud_down_body->points.push_back(p_body);
        }
        return cloud_down_body;
    }
    
    void LIOBuilder::reset()
    {

        status = Status::INITIALIZE;
        imu_processor_->reset();
        state_ikfom state = kf_->get_x();
        state.rot.setIdentity();
        state.pos.setZero();
        state.vel.setZero();
        state.offset_R_L_I.setIdentity();
        state.offset_T_L_I.setZero();
        state.ba.setZero();
        state.bg.setZero();
        kf_->change_x(state);
        ikdtree_.reset(new KD_TREE<PointType>);
        configureIKDTree();
        local_map_.is_initialed = false;
        processed_lidar_frames_ = 0;
        resetWheelOdom();
    }

    void LIOBuilder::pushWheelOdom(const nav_msgs::Odometry::ConstPtr &odomMsg)
    {
        if (!use_wheel_odom_)
            return;
        if (!odomMsg)
            return;
        WheelOdomData data;
        data.stamp = odomMsg->header.stamp.toSec();
        data.pos << odomMsg->pose.pose.position.x,
            odomMsg->pose.pose.position.y,
            odomMsg->pose.pose.position.z;
        data.vel << odomMsg->twist.twist.linear.x,
            odomMsg->twist.twist.linear.y,
            odomMsg->twist.twist.linear.z;
        data.angular_vel << odomMsg->twist.twist.angular.x,
            odomMsg->twist.twist.angular.y,
            odomMsg->twist.twist.angular.z;

        const bool use_angular_twist = odom_fuse_angular_velocity_;
        if (!std::isfinite(data.stamp) || !data.vel.allFinite() ||
            (use_angular_twist && !data.angular_vel.allFinite()) ||
            (odom_max_linear_speed_ > 0.0 &&
             data.vel.norm() > odom_max_linear_speed_) ||
            (use_angular_twist && odom_max_angular_speed_ > 0.0 &&
             data.angular_vel.norm() > odom_max_angular_speed_))
        {
            ROS_WARN_THROTTLE(2.0, "discarding invalid wheel odometry twist");
            return;
        }
        if (data.vel.head<2>().norm() < odom_linear_deadband_)
            data.vel.head<2>().setZero();
        if (use_angular_twist)
        {
            if (std::abs(data.angular_vel.z()) < odom_angular_deadband_)
                data.angular_vel.z() = 0.0;
        }
        else
        {
            // The configured data source may copy the already-consumed IMU
            // gyro into nav_msgs/Odometry. Do not let an unused copy reject a
            // valid, independent linear-velocity observation.
            data.angular_vel.setZero();
        }

        if (odomMsg->pose.pose.orientation.w != 0.0 ||
            odomMsg->pose.pose.orientation.x != 0.0 ||
            odomMsg->pose.pose.orientation.y != 0.0 ||
            odomMsg->pose.pose.orientation.z != 0.0)
        {
            Eigen::Quaterniond q_odom(
                odomMsg->pose.pose.orientation.w,
                odomMsg->pose.pose.orientation.x,
                odomMsg->pose.pose.orientation.y,
                odomMsg->pose.pose.orientation.z);
            data.rot = q_odom.normalized().toRotationMatrix() * R_odom_to_imu_.transpose();
            data.has_pose = true;
        }
        else
        {
            data.has_pose = false;
        }

        if (!odom_velocity_in_body_frame_)
        {
            if (!data.has_pose)
            {
                ROS_WARN_THROTTLE(2.0, "wheel velocity is in the odometry frame but orientation is missing; discarding sample");
                return;
            }
            Eigen::Quaterniond q_odom(
                odomMsg->pose.pose.orientation.w,
                odomMsg->pose.pose.orientation.x,
                odomMsg->pose.pose.orientation.y,
                odomMsg->pose.pose.orientation.z);
            data.vel = q_odom.normalized().conjugate() * data.vel;
        }

        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        if (!wheel_odom_buffer_.empty() &&
            data.stamp <= wheel_odom_buffer_.back().stamp)
        {
            if (data.stamp + 1e-3 < wheel_odom_buffer_.back().stamp)
            {
                ROS_WARN("wheel odometry timestamp moved backwards; clearing motion evidence");
                wheel_odom_buffer_.clear();
                has_last_wheel_time_ = false;
                last_wheel_odom_frame_time_ = 0.0;
            }
            else
            {
                ROS_WARN_THROTTLE(2.0, "discarding duplicate or slightly out-of-order wheel odometry");
                return;
            }
        }
        wheel_odom_buffer_.push_back(data);
        while (wheel_odom_buffer_.size() > wheel_odom_max_buf_)
        {
            wheel_odom_buffer_.pop_front();
        }
    }

    void LIOBuilder::resetWheelOdom()
    {
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        wheel_odom_buffer_.clear();
        has_last_wheel_state_ = false;
        has_last_wheel_time_ = false;
        has_wheel_odom_z_ref_ = false;
        wheel_odom_z_ref_ = 0.0;
        slam_z_ref_ = 0.0;
        has_last_odom_height_ = false;
        has_odom_orientation_anchor_ = false;
        R_map_odom_anchor_.setIdentity();
        last_odom_height_ = 0.0;
        last_wheel_odom_frame_time_ = 0.0;
    }

    bool LIOBuilder::predictStateWheelOdom(double frame_begin, double frame_end,
                                           const Eigen::Matrix3d &imu_pred_rot,
                                           state_ikfom &state_out,
                                           bool &has_orientation)
    {
        has_orientation = false;
        if (!use_wheel_odom_ || !has_last_wheel_state_)
            return false;
        if (frame_end <= frame_begin)
            return false;
        if (frame_end - frame_begin > 1.0)
            return false;

        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        if (wheel_odom_buffer_.empty())
            return false;

        double odom_start_time = has_last_wheel_time_ ? last_wheel_odom_frame_time_ : frame_begin;
        double dt = frame_end - odom_start_time;
        if (dt <= 1e-4 || dt > 1.0)
            return false;

        Eigen::Matrix3d R_odom_start = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d R_odom_end = Eigen::Matrix3d::Identity();
        double odom_z_end = 0.0;
        bool has_start_rot = false;
        bool has_end_rot = false;
        bool has_end_pos = false;
        for (const auto &m : wheel_odom_buffer_)
        {
            if (m.stamp <= odom_start_time && m.has_pose)
            {
                R_odom_start = m.rot;
                has_start_rot = true;
            }
            if (m.stamp <= frame_end && m.has_pose)
            {
                R_odom_end = m.rot;
                odom_z_end = m.pos.z();
                has_end_pos = true;
                has_end_rot = true;
            }
            if (m.stamp > frame_end)
                break;
        }

        while (wheel_odom_buffer_.size() > 1 && wheel_odom_buffer_[1].stamp < odom_start_time)
        {
            wheel_odom_buffer_.pop_front();
        }
        if (wheel_odom_buffer_.empty())
            return false;

        std::vector<WheelOdomData> valid;
        for (const auto &m : wheel_odom_buffer_)
        {
            if (m.stamp < odom_start_time)
                continue;
            if (m.stamp > frame_end)
                break;
            valid.push_back(m);
        }
        if (valid.empty())
            return false;

        Eigen::Vector3d vel_sum = Eigen::Vector3d::Zero();
        Eigen::Vector3d angular_vel_sum = Eigen::Vector3d::Zero();
        for (const auto &m : valid)
        {
            vel_sum += m.vel;
            angular_vel_sum += m.angular_vel;
        }
        Eigen::Vector3d avg_vel_odom = vel_sum / static_cast<double>(valid.size());
        Eigen::Vector3d avg_angular_vel_odom =
            angular_vel_sum / static_cast<double>(valid.size());

        Eigen::Vector3d avg_vel_tail = Eigen::Vector3d::Zero();
        int tail_cnt = 0;
        for (int i = static_cast<int>(valid.size()) - 1; i >= 0 && tail_cnt < 2; --i)
        {
            avg_vel_tail += valid[i].vel;
            ++tail_cnt;
        }
        avg_vel_tail /= static_cast<double>(tail_cnt);
        if (odom_planar_)
        {
            avg_vel_odom.z() = 0.0;
            avg_vel_tail.z() = 0.0;
            avg_angular_vel_odom.x() = 0.0;
            avg_angular_vel_odom.y() = 0.0;
        }

        Eigen::Vector3d avg_vel_imu_for_pos = R_odom_to_imu_ * avg_vel_odom;
        Eigen::Vector3d avg_vel_imu_for_vel = R_odom_to_imu_ * avg_vel_tail;

        Eigen::Matrix3d R_last = state_out.rot.toRotationMatrix();
        if (odom_fuse_integrated_position_)
        {
            const Eigen::Vector3d delta_body = avg_vel_imu_for_pos * dt;
            state_out.pos += R_last * delta_body;
        }

        if (odom_use_orientation_ && has_end_rot && has_start_rot)
        {
            if (odom_use_absolute_orientation_)
            {
                if (!has_odom_orientation_anchor_)
                {
                    R_map_odom_anchor_ =
                        state_out.rot.toRotationMatrix() * R_odom_end.transpose();
                    has_odom_orientation_anchor_ = true;
                }
                state_out.rot = MTK::SO3<double>(
                    R_map_odom_anchor_ * R_odom_end);
                has_orientation = true;
            }
            else
            {
                Eigen::Matrix3d R_delta =
                    R_odom_start.transpose() * R_odom_end;
                Eigen::AngleAxisd aa(R_delta);
                double rot_angle = aa.angle();
                double max_rot = std::min(
                    90.0 * M_PI / 180.0,
                    180.0 * M_PI / 180.0 * dt);
                if (rot_angle < max_rot)
                {
                    Eigen::Matrix3d R_updated =
                        state_out.rot.toRotationMatrix() * R_delta;
                    state_out.rot = MTK::SO3<double>(R_updated);
                    has_orientation = true;
                }
            }
        }
        if (!has_orientation && odom_fuse_angular_velocity_)
        {
            const Eigen::Vector3d avg_angular_vel_imu =
                R_odom_to_imu_ * avg_angular_vel_odom;
            const Eigen::Vector3d delta_angle = avg_angular_vel_imu * dt;
            Eigen::Matrix3d R_delta = Eigen::Matrix3d::Identity();
            if (delta_angle.norm() > 1e-12)
            {
                R_delta = Eigen::AngleAxisd(
                              delta_angle.norm(), delta_angle.normalized())
                              .toRotationMatrix();
            }
            const Eigen::Matrix3d R_updated =
                state_out.rot.toRotationMatrix() * R_delta;
            state_out.rot = MTK::SO3<double>(R_updated);
            has_orientation = true;
        }
        // twist.linear is expressed in the current body frame. If no
        // independent odometry orientation is enabled, use the current IMU
        // prediction rather than the previous LiDAR frame's orientation;
        // otherwise turns introduce a systematic one-frame velocity lag.
        const Eigen::Matrix3d velocity_world_rotation =
            has_orientation ? state_out.rot.toRotationMatrix() : imu_pred_rot;
        state_out.vel = velocity_world_rotation * avg_vel_imu_for_vel;
        if (odom_planar_ && odom_constrain_z_ && has_end_pos)
        {
            if (!has_wheel_odom_z_ref_)
            {
                wheel_odom_z_ref_ = odom_z_end;
                slam_z_ref_ = last_state_point_wheel_.pos.z();
                has_wheel_odom_z_ref_ = true;
            }
            state_out.pos.z() = slam_z_ref_ + (odom_z_end - wheel_odom_z_ref_);
            state_out.vel.z() = 0.0;
            last_odom_height_ = state_out.pos.z();
            has_last_odom_height_ = true;
        }

        last_wheel_odom_frame_time_ = frame_end;
        has_last_wheel_time_ = true;
        return true;
    }

    void LIOBuilder::applyWheelOdomUpdate(const MeasureGroup &meas, const Eigen::Matrix3d &imu_pred_rot)
    {
        if (!use_wheel_odom_)
            return;
        if (!has_last_wheel_state_)
            return;
        if (status == Status::INITIALIZE)
            return;

        state_ikfom odom_state = last_state_point_wheel_;
        bool has_odom_orientation = false;
        if (!predictStateWheelOdom(meas.lidar_time_begin, meas.lidar_time_end,
                                   imu_pred_rot, odom_state,
                                   has_odom_orientation))
        {
            ROS_WARN_THROTTLE(2.0, "wheel odometry is unavailable at the lidar timestamp; skipping its ESKF update");
            return;
        }

        state_ikfom state_point = kf_->get_x();
        auto P_imu = kf_->get_P();

        bool use_orientation = has_odom_orientation;
        double rot_innovation = 0.0;
        if (use_orientation && odom_rot_innovation_gate_ > 0.0)
        {
            const Eigen::Matrix3d last_rot = last_state_point_wheel_.rot.toRotationMatrix();
            const Eigen::Matrix3d odom_delta = last_rot.transpose() * odom_state.rot.toRotationMatrix();
            const Eigen::Matrix3d imu_delta = last_rot.transpose() * imu_pred_rot;
            rot_innovation = Eigen::AngleAxisd(imu_delta.transpose() * odom_delta).angle();
            if (rot_innovation > odom_rot_innovation_gate_)
                use_orientation = false;
        }
        // A2 odom_lf has useful RFU XY velocity but a demonstrably drifting
        // twist.z channel.  Keep this independent from `planar`: disabling a
        // bad Z measurement must not impose a flat-ground state constraint.
        const bool use_z =
            (!odom_planar_ || odom_constrain_z_) && odom_fuse_z_velocity_;
        const bool directional_odom =
            hdl_directional_constraint_ && lidar_planar_degenerate_ &&
            lidar_planar_observability_available_;
        // Directional separation applies in map XY. Keep the pre-existing Z
        // odometry row identical in both A/B groups when it is configured.
        const int pos_dim = directional_odom ? (use_z ? 2 : 1)
                                             : (use_z ? 3 : 2);
        const int vel_dim = directional_odom ? (use_z ? 2 : 1)
                                             : (use_z ? 3 : 2);
        const int rot_dim = use_orientation ? 3 : 0;
        const int fused_pos_dim = odom_fuse_integrated_position_ ? pos_dim : 0;
        const int fused_vel_dim = odom_fuse_velocity_ ? vel_dim : 0;
        const int meas_dim = rot_dim + fused_pos_dim + fused_vel_dim;
        if (meas_dim == 0)
            return;

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(meas_dim, 23);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(meas_dim, meas_dim);
        Eigen::VectorXd residual = Eigen::VectorXd::Zero(meas_dim);

        int row = 0;
        if (use_orientation)
        {
            Eigen::Matrix<double, 3, 1> rot_residual;
            odom_state.rot.boxminus(rot_residual, state_point.rot);
            residual.segment<3>(row) = rot_residual;
            H.block<3, 3>(row, 3) = Eigen::Matrix3d::Identity();
            R.block<3, 3>(row, row) = Eigen::Matrix3d::Identity() * (odom_rot_noise_std_ * odom_rot_noise_std_);
            row += 3;
        }

        if (odom_fuse_integrated_position_)
        {
            if (directional_odom)
            {
                const Eigen::Vector2d weak = lidar_weak_direction_.normalized();
                H.block<1, 2>(row, 0) = weak.transpose();
                residual(row) = weak.dot(
                    (odom_state.pos - state_point.pos).head<2>());
                if (use_z)
                {
                    H(row + 1, 2) = 1.0;
                    residual(row + 1) = odom_state.pos.z() - state_point.pos.z();
                }
            }
            else
            {
                H.block(row, 0, pos_dim, pos_dim) =
                    Eigen::MatrixXd::Identity(pos_dim, pos_dim);
                residual.segment(row, pos_dim) =
                    (odom_state.pos - state_point.pos).head(pos_dim);
            }
            R.block(row, row, pos_dim, pos_dim) =
                Eigen::MatrixXd::Identity(pos_dim, pos_dim) *
                (odom_pos_noise_std_ * odom_pos_noise_std_);
            if (use_z)
                R(row + pos_dim - 1, row + pos_dim - 1) =
                    odom_z_pos_noise_std_ * odom_z_pos_noise_std_;
            row += pos_dim;
        }

        if (odom_fuse_velocity_)
        {
            if (directional_odom)
            {
                const Eigen::Vector2d weak = lidar_weak_direction_.normalized();
                H.block<1, 2>(row, 12) = weak.transpose();
                residual(row) = weak.dot(
                    (odom_state.vel - state_point.vel).head<2>());
                if (use_z)
                {
                    H(row + 1, 14) = 1.0;
                    residual(row + 1) = odom_state.vel.z() - state_point.vel.z();
                }
            }
            else
            {
                H.block(row, 12, vel_dim, vel_dim) =
                    Eigen::MatrixXd::Identity(vel_dim, vel_dim);
                residual.segment(row, vel_dim) =
                    (odom_state.vel - state_point.vel).head(vel_dim);
            }
            R.block(row, row, vel_dim, vel_dim) =
                Eigen::MatrixXd::Identity(vel_dim, vel_dim) *
                (odom_vel_noise_std_ * odom_vel_noise_std_);
            if (use_z)
                R(row + vel_dim - 1, row + vel_dim - 1) =
                    odom_z_vel_noise_std_ * odom_z_vel_noise_std_;
        }

        if (odom_rot_covariance_floor_scale_ > 0.0)
        {
            for (int i = 0; i < rot_dim; ++i)
            {
                Eigen::Index state_index = 0;
                H.row(i).cwiseAbs().maxCoeff(&state_index);
                const double covariance_floor =
                    odom_rot_covariance_floor_scale_ * R(i, i);
                if (P_imu(state_index, state_index) < covariance_floor)
                    P_imu(state_index, state_index) = covariance_floor;
            }
        }

        if (odom_covariance_floor_scale_ > 0.0)
        {
            for (int i = rot_dim; i < meas_dim; ++i)
            {
                Eigen::Index state_index = 0;
                H.row(i).cwiseAbs().maxCoeff(&state_index);
                const double covariance_floor =
                    odom_covariance_floor_scale_ * R(i, i);
                if (P_imu(state_index, state_index) < covariance_floor)
                {
                    P_imu(state_index, state_index) = covariance_floor;
                }
            }
        }

        Eigen::MatrixXd S = H * P_imu * H.transpose() + R;
        Eigen::MatrixXd K = P_imu * H.transpose() * S.inverse();

        Eigen::Matrix<double, 23, 1> state_update = K * residual;
        Eigen::Vector3d pos_before = state_point.pos;
        Eigen::Vector3d vel_before = state_point.vel;
        Eigen::Matrix3d rot_before = state_point.rot.toRotationMatrix();
        state_point.boxplus(state_update);

        Eigen::Matrix<double, 23, 23> I_KH = Eigen::Matrix<double, 23, 23>::Identity() - K * H;
        Eigen::Matrix<double, 23, 23> P_updated = I_KH * P_imu * I_KH.transpose() + K * R * K.transpose();
        P_updated = 0.5 * (P_updated + P_updated.transpose());

        if (odom_debug_)
        {
            std::ostringstream oss;
            Eigen::Matrix3d rot_after = state_point.rot.toRotationMatrix();
            const double rot_update_angle =
                Eigen::AngleAxisd(rot_before.transpose() * rot_after).angle();
            oss << "\n[WheelOdom EKF] meas(rot_used,pos,vel)=" << use_orientation << ", "
                << odom_state.pos.transpose() << ", " << odom_state.vel.transpose()
                << "\n[WheelOdom EKF] rot_innovation=" << rot_innovation
                << "\n[WheelOdom EKF] state_before(pos,vel)=" << pos_before.transpose() << ", " << vel_before.transpose()
                << "\n[WheelOdom EKF] rot_delta_trace=" << (rot_before.transpose() * rot_after).trace()
                << " rot_update_angle=" << rot_update_angle
                << "\n[WheelOdom EKF] residual=" << residual.transpose()
                << "\n[WheelOdom EKF] state_after(pos,vel)=" << state_point.pos.transpose() << ", " << state_point.vel.transpose();
            ROS_INFO_STREAM(oss.str());
        }

        kf_->change_x(state_point);
        kf_->change_P(P_updated);
    }

    void LIOBuilder::applyPlanarHeightConstraint()
    {
        if (!odom_planar_ || !odom_constrain_z_ ||
            !odom_hard_constrain_z_ || !has_last_odom_height_)
            return;

        state_ikfom state_point = kf_->get_x();
        state_point.pos.z() = last_odom_height_;
        state_point.vel.z() = 0.0;
        kf_->change_x(state_point);

        auto P = kf_->get_P();
        P.row(2).setZero();
        P.col(2).setZero();
        P.row(14).setZero();
        P.col(14).setZero();
        P(2, 2) = odom_z_pos_noise_std_ * odom_z_pos_noise_std_;
        P(14, 14) = odom_z_vel_noise_std_ * odom_z_vel_noise_std_;
        kf_->change_P(P);
    }
}
