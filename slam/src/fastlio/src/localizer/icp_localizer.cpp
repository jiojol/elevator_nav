#include "localizer/icp_localizer.h"
// #include <chrono>

#include <cmath>

namespace fastlio
{
    bool IcpLocalizer::init(const std::string &pcd_path, bool with_norm)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        success_ = false;
        initialized_ = false;
        score_ = std::numeric_limits<double>::infinity();
        diagnostics_ = RegistrationDiagnostics();
        pcd_path_.clear();
        {
            std::lock_guard<std::mutex> map_lock(map_mutex_);
            refine_map_.reset();
            rough_map_.reset();
        }

        if (pcd_path.empty())
        {
            ROS_ERROR("Cannot initialize ICP: PCD path is empty.");
            return false;
        }

        pcl::PCDReader reader;
        PointCloudXYZI::Ptr refine_map_candidate;
        if (!with_norm)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            if (reader.read(pcd_path, *cloud) < 0 || cloud->empty())
            {
                ROS_ERROR_STREAM("Cannot initialize ICP: failed to read non-empty PCD " << pcd_path);
                return false;
            }
            voxel_refine_filter_.setInputCloud(cloud);
            voxel_refine_filter_.filter(*cloud);
            if (cloud->empty())
            {
                ROS_ERROR_STREAM("Cannot initialize ICP: refine map is empty after filtering " << pcd_path);
                return false;
            }
            refine_map_candidate = addNorm(cloud);
        }
        else
        {
            refine_map_candidate.reset(new PointCloudXYZI);
            if (reader.read(pcd_path, *refine_map_candidate) < 0 || refine_map_candidate->empty())
            {
                ROS_ERROR_STREAM("Cannot initialize ICP: failed to read non-empty normal PCD " << pcd_path);
                return false;
            }
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr point_rough(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_point_rough(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*refine_map_candidate, *point_rough);
        voxel_rough_filter_.setInputCloud(point_rough);
        voxel_rough_filter_.filter(*filterd_point_rough);
        if (filterd_point_rough->empty())
        {
            ROS_ERROR_STREAM("Cannot initialize ICP: rough map is empty after filtering " << pcd_path);
            return false;
        }
        PointCloudXYZI::Ptr rough_map_candidate = addNorm(filterd_point_rough);

        icp_rough_.setMaximumIterations(rough_iter_);
        icp_rough_.setInputTarget(rough_map_candidate);

        icp_refine_.setMaximumIterations(refine_iter_);
        icp_refine_.setInputTarget(refine_map_candidate);
        {
            std::lock_guard<std::mutex> map_lock(map_mutex_);
            refine_map_ = refine_map_candidate;
            rough_map_ = rough_map_candidate;
        }
        pcd_path_ = pcd_path;
        initialized_ = true;
        return true;
    }

    Eigen::Matrix4d IcpLocalizer::align(pcl::PointCloud<pcl::PointXYZI>::Ptr source, Eigen::Matrix4d init_guess)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        success_ = false;
        score_ = std::numeric_limits<double>::infinity();
        diagnostics_ = RegistrationDiagnostics();
        if (!initialized_ || !source || source->empty() || !init_guess.allFinite())
            return Eigen::Matrix4d::Zero();
        Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);

        pcl::PointCloud<pcl::PointXYZI>::Ptr rough_source(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(new pcl::PointCloud<pcl::PointXYZI>);

        voxel_rough_filter_.setInputCloud(source);
        voxel_rough_filter_.filter(*rough_source);
        voxel_refine_filter_.setInputCloud(source);
        voxel_refine_filter_.filter(*refine_source);

        PointCloudXYZI::Ptr rough_source_norm = addNorm(rough_source);
        PointCloudXYZI::Ptr refine_source_norm = addNorm(refine_source);
        PointCloudXYZI::Ptr align_point(new PointCloudXYZI);
        // auto tic = std::chrono::system_clock::now();
        icp_rough_.setInputSource(rough_source_norm);
        icp_rough_.align(*align_point, init_guess.cast<float>());

        score_ = icp_rough_.getFitnessScore();
        if (!icp_rough_.hasConverged())
            return Eigen::Matrix4d::Zero();

        icp_refine_.setInputSource(refine_source_norm);
        icp_refine_.align(*align_point, icp_rough_.getFinalTransformation());
        score_ = icp_refine_.getFitnessScore();

        if (!icp_refine_.hasConverged())
            return Eigen::Matrix4d::Zero();
        computeDiagnostics(refine_source_norm, icp_refine_.getFinalTransformation());
        if (score_ > thresh_)
            return Eigen::Matrix4d::Zero();
        success_ = true;
        // auto toc = std::chrono::system_clock::now();
        // std::chrono::duration<double> duration = toc - tic;
        // std::cout << "align used: " << duration.count() * 1000 << std::endl;
        return icp_refine_.getFinalTransformation().cast<double>();
    }

    PointCloudXYZI::Ptr IcpLocalizer::addNorm(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZI>);
        searchTree->setInputCloud(cloud);

        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
        normalEstimator.setInputCloud(cloud);
        normalEstimator.setSearchMethod(searchTree);
        normalEstimator.setKSearch(15);
        normalEstimator.compute(*normals);
        PointCloudXYZI::Ptr out(new PointCloudXYZI);
        pcl::concatenateFields(*cloud, *normals, *out);
        return out;
    }

    void IcpLocalizer::writePCDToFile(const std::string &path, bool detail)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_)
            return;
        pcl::PCDWriter writer;
        writer.writeBinaryCompressed(path, detail ? *refine_map_ : *rough_map_);
    }

    void IcpLocalizer::setParams(double refine_resolution, double rough_resolution, int refine_iter, int rough_iter, double thresh)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        refine_resolution_ = refine_resolution;
        rough_resolution_ = rough_resolution;
        refine_iter_ = refine_iter;
        rough_iter_ = rough_iter;
        thresh_ = thresh;
    }

    void IcpLocalizer::setSearchParams(double xy_offset, int yaw_offset, double yaw_res){
        std::lock_guard<std::mutex> lock(mutex_);
        xy_offset_ = xy_offset;
        yaw_offset_ = yaw_offset;
        yaw_resolution_ = yaw_res;
    }

    void IcpLocalizer::setDiagnosticParams(double max_correspondence_distance, double max_valid_point_distance)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        diagnostic_max_correspondence_distance_ = std::max(0.01, max_correspondence_distance);
        diagnostic_max_valid_point_distance_ = std::max(0.0, max_valid_point_distance);
    }

    void IcpLocalizer::computeDiagnostics(const PointCloudXYZI::ConstPtr &source, const Eigen::Matrix4f &transform)
    {
        diagnostics_ = RegistrationDiagnostics();
        if (!source || source->empty() || !refine_map_ || refine_map_->empty() || !transform.allFinite())
            return;

        const auto search = icp_refine_.getSearchMethodTarget();
        if (!search)
            return;

        const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0).cast<double>();
        const Eigen::Vector3d translation = transform.block<3, 1>(0, 3).cast<double>();
        const double max_correspondence_sq = diagnostic_max_correspondence_distance_ * diagnostic_max_correspondence_distance_;
        const double max_range_sq = diagnostic_max_valid_point_distance_ * diagnostic_max_valid_point_distance_;
        double squared_error_sum = 0.0;
        std::vector<int> indices(1);
        std::vector<float> squared_distances(1);

        for (const PointType &source_point : source->points)
        {
            const Eigen::Vector3d point_body(source_point.x, source_point.y, source_point.z);
            if (!point_body.allFinite())
                continue;
            if (diagnostic_max_valid_point_distance_ > 0.0 && point_body.squaredNorm() > max_range_sq)
                continue;

            ++diagnostics_.valid_points;
            const Eigen::Vector3d point_map = rotation * point_body + translation;
            PointType query{};
            query.x = static_cast<float>(point_map.x());
            query.y = static_cast<float>(point_map.y());
            query.z = static_cast<float>(point_map.z());
            if (search->nearestKSearch(query, 1, indices, squared_distances) <= 0 ||
                indices.empty() || squared_distances.empty() ||
                indices.front() < 0 ||
                static_cast<std::size_t>(indices.front()) >= refine_map_->size() ||
                !std::isfinite(squared_distances.front()) ||
                squared_distances.front() >= max_correspondence_sq)
            {
                continue;
            }

            const PointType &target = refine_map_->points[indices.front()];
            Eigen::Vector3d normal(target.normal_x, target.normal_y, target.normal_z);
            if (!normal.allFinite() || normal.norm() < 1e-6)
                continue;
            normal.normalize();

            Eigen::Matrix<double, 1, 6> jacobian;
            jacobian.head<3>() = normal.transpose();
            jacobian.tail<3>() = point_map.cross(normal).transpose();
            diagnostics_.hessian.noalias() += jacobian.transpose() * jacobian;
            squared_error_sum += squared_distances.front();
            ++diagnostics_.inliers;
        }

        diagnostics_.inlier_fraction = static_cast<double>(diagnostics_.inliers) /
            static_cast<double>(std::max(1, diagnostics_.valid_points));
        if (diagnostics_.inliers > 0)
            diagnostics_.matching_error = squared_error_sum / static_cast<double>(diagnostics_.inliers);
        diagnostics_.available = diagnostics_.inliers >= 6 && diagnostics_.hessian.allFinite();
    }

    Eigen::Matrix4d IcpLocalizer::multi_align_sync(pcl::PointCloud<pcl::PointXYZI>::Ptr source, Eigen::Matrix4d init_guess)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        success_ = false;
        score_ = std::numeric_limits<double>::infinity();
        diagnostics_ = RegistrationDiagnostics();
        if (!initialized_ || !source || source->empty() || !init_guess.allFinite())
            return Eigen::Matrix4d::Zero();
        Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);
        Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);
        Eigen::Vector3d rpy = rotate2rpy(rotation);
        Eigen::AngleAxisf rollAngle(rpy(0), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(rpy(1), Eigen::Vector3f::UnitY());
        std::vector<Eigen::Matrix4f> candidates;
        Eigen::Matrix4f temp_pose;

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                for (int k = -yaw_offset_; k <= yaw_offset_; k++)
                {
                    Eigen::Vector3f pos(xyz(0) + i * xy_offset_, xyz(1) + j * xy_offset_, xyz(2));
                    Eigen::AngleAxisf yawAngle(rpy(2) + k * yaw_resolution_, Eigen::Vector3f::UnitZ());
                    temp_pose.setIdentity();
                    temp_pose.block<3, 3>(0, 0) =
                        (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
                    temp_pose.block<3, 1>(0, 3) = pos;
                    candidates.push_back(temp_pose);
                }
            }
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr rough_source(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(new pcl::PointCloud<pcl::PointXYZI>);

        voxel_rough_filter_.setInputCloud(source);
        voxel_rough_filter_.filter(*rough_source);
        voxel_refine_filter_.setInputCloud(source);
        voxel_refine_filter_.filter(*refine_source);

        PointCloudXYZI::Ptr rough_source_norm = addNorm(rough_source);
        PointCloudXYZI::Ptr refine_source_norm = addNorm(refine_source);
        PointCloudXYZI::Ptr align_point(new PointCloudXYZI);

        Eigen::Matrix4f best_rough_transform;
        double best_rough_score = 10.0;
        bool rough_converge = false;
        // auto tic = std::chrono::system_clock::now();
        for (Eigen::Matrix4f &init_pose : candidates)
        {
            icp_rough_.setInputSource(rough_source_norm);
            icp_rough_.align(*align_point, init_pose);
            if (!icp_rough_.hasConverged())
                continue;
            double rough_score = icp_rough_.getFitnessScore();
            if (rough_score > 2 * thresh_)
                continue;
            if (rough_score < best_rough_score)
            {
                best_rough_score = rough_score;
                rough_converge = true;
                best_rough_transform = icp_rough_.getFinalTransformation();
            }
        }

        if (!rough_converge)
            return Eigen::Matrix4d::Zero();

        icp_refine_.setInputSource(refine_source_norm);
        icp_refine_.align(*align_point, best_rough_transform);
        score_ = icp_refine_.getFitnessScore();

        if (!icp_refine_.hasConverged())
            return Eigen::Matrix4d::Zero();
        computeDiagnostics(refine_source_norm, icp_refine_.getFinalTransformation());
        if (score_ > thresh_)
            return Eigen::Matrix4d::Zero();
        success_ = true;
        // auto toc = std::chrono::system_clock::now();
        // std::chrono::duration<double> duration = toc - tic;
        // std::cout << "multi_align used: " << duration.count() * 1000 << std::endl;
        return icp_refine_.getFinalTransformation().cast<double>();
    }
}
