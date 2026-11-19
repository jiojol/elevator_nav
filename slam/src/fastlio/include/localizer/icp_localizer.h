#pragma once
#include <algorithm>
#include <limits>
#include <mutex>
#include <string>
#include "commons.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace fastlio
{

    struct LocalizerParams
    {
        double rough_resolution = 0.5;
        double refine_resolution = 0.2;
        double rough_iter = 10;
        double refine_iter = 5;
        double thresh = 0.15;
        double xy_offset = 2.0;
        int yaw_offset = 3;
        double yaw_resolution = M_PI / 6;
    };

    struct RegistrationDiagnostics
    {
        // Gauss-Newton order is [tx, ty, tz, rx, ry, rz], using a left
        // perturbation in the map frame. matching_error is mean squared
        // Euclidean nearest-neighbour distance in m^2.
        bool available = false;
        int valid_points = 0;
        int inliers = 0;
        double inlier_fraction = 0.0;
        double matching_error = std::numeric_limits<double>::infinity();
        Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
    };

    class IcpLocalizer
    {
    public:
        IcpLocalizer()
            : refine_resolution_(0.2),
              rough_resolution_(0.5),
              rough_iter_(10),
              refine_iter_(5),
              thresh_(0.15)
        {
            voxel_rough_filter_.setLeafSize(rough_resolution_, rough_resolution_, rough_resolution_);
            voxel_refine_filter_.setLeafSize(refine_resolution_, refine_resolution_, refine_resolution_);
        }
        IcpLocalizer(double refine_resolution, double rough_resolution, int refine_iter, int rough_iter, double thresh)
            : refine_resolution_(refine_resolution),
              rough_resolution_(rough_resolution),
              refine_iter_(refine_iter),
              rough_iter_(rough_iter),
              thresh_(thresh)
        {
            voxel_rough_filter_.setLeafSize(rough_resolution_, rough_resolution_, rough_resolution_);
            voxel_refine_filter_.setLeafSize(refine_resolution_, refine_resolution_, refine_resolution_);
        }
        double getScore() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return score_;
        }

        RegistrationDiagnostics getDiagnostics() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return diagnostics_;
        }

        bool isSuccess() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return success_;
        }

        bool isInitialized() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return initialized_;
        }

        bool init(const std::string &pcd_path, bool with_norm);

        static PointCloudXYZI::Ptr addNorm(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

        Eigen::Matrix4d align(pcl::PointCloud<pcl::PointXYZI>::Ptr source, Eigen::Matrix4d init_guess);

        Eigen::Matrix4d multi_align_sync(pcl::PointCloud<pcl::PointXYZI>::Ptr source, Eigen::Matrix4d init_guess);

        PointCloudXYZI::Ptr getRoughMap() const
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            return rough_map_;
        }

        PointCloudXYZI::Ptr getRefineMap() const
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            return refine_map_;
        }

        void writePCDToFile(const std::string &path, bool detail);

        void setParams(double refine_resolution, double rough_resolution, int refine_iter, int rough_iter, double thresh);

        void setSearchParams(double xy_offset, int yaw_offset, double yaw_res);

        void setDiagnosticParams(double max_correspondence_distance, double max_valid_point_distance);

    private:
        std::string pcd_path_;
        PointCloudXYZI::Ptr refine_map_;
        PointCloudXYZI::Ptr rough_map_;
        double refine_resolution_;
        double rough_resolution_;
        int rough_iter_;
        int refine_iter_;
        double thresh_;
        double score_ = 10.0;
        bool success_ = false;
        bool initialized_ = false;
        double xy_offset_ = 2.0;
        int yaw_offset_ = 3;
        double yaw_resolution_ = M_PI / 6;
        pcl::VoxelGrid<pcl::PointXYZI> voxel_rough_filter_;
        pcl::VoxelGrid<pcl::PointXYZI> voxel_refine_filter_;
        pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_rough_;
        pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_refine_;
        RegistrationDiagnostics diagnostics_;
        double diagnostic_max_correspondence_distance_ = 0.5;
        double diagnostic_max_valid_point_distance_ = 25.0;
        mutable std::mutex mutex_;
        mutable std::mutex map_mutex_;

        void computeDiagnostics(const PointCloudXYZI::ConstPtr &source, const Eigen::Matrix4f &transform);
    };
}
