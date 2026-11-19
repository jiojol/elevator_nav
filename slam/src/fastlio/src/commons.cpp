#include "commons.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace
{
bool fieldToFloat(const uint8_t *data, const sensor_msgs::PointField &field, float &value)
{
    switch (field.datatype)
    {
    case sensor_msgs::PointField::FLOAT32:
        std::memcpy(&value, data + field.offset, sizeof(float));
        return true;
    case sensor_msgs::PointField::FLOAT64:
    {
        double temp = 0.0;
        std::memcpy(&temp, data + field.offset, sizeof(double));
        value = static_cast<float>(temp);
        return true;
    }
    case sensor_msgs::PointField::UINT8:
        value = static_cast<float>(*(data + field.offset));
        return true;
    case sensor_msgs::PointField::INT8:
        value = static_cast<float>(*reinterpret_cast<const int8_t *>(data + field.offset));
        return true;
    case sensor_msgs::PointField::UINT16:
    {
        uint16_t temp = 0;
        std::memcpy(&temp, data + field.offset, sizeof(uint16_t));
        value = static_cast<float>(temp);
        return true;
    }
    case sensor_msgs::PointField::INT16:
    {
        int16_t temp = 0;
        std::memcpy(&temp, data + field.offset, sizeof(int16_t));
        value = static_cast<float>(temp);
        return true;
    }
    case sensor_msgs::PointField::UINT32:
    {
        uint32_t temp = 0;
        std::memcpy(&temp, data + field.offset, sizeof(uint32_t));
        value = static_cast<float>(temp);
        return true;
    }
    case sensor_msgs::PointField::INT32:
    {
        int32_t temp = 0;
        std::memcpy(&temp, data + field.offset, sizeof(int32_t));
        value = static_cast<float>(temp);
        return true;
    }
    default:
        return false;
    }
}

bool fieldToDouble(const uint8_t *data, const sensor_msgs::PointField &field, double &value)
{
    switch (field.datatype)
    {
    case sensor_msgs::PointField::FLOAT64:
        std::memcpy(&value, data + field.offset, sizeof(double));
        return true;
    case sensor_msgs::PointField::FLOAT32:
    {
        float temp = 0.0f;
        std::memcpy(&temp, data + field.offset, sizeof(float));
        value = static_cast<double>(temp);
        return true;
    }
    case sensor_msgs::PointField::UINT8:
        value = static_cast<double>(*(data + field.offset));
        return true;
    case sensor_msgs::PointField::INT8:
        value = static_cast<double>(*reinterpret_cast<const int8_t *>(data + field.offset));
        return true;
    case sensor_msgs::PointField::UINT16:
    {
        uint16_t temp = 0;
        std::memcpy(&temp, data + field.offset, sizeof(uint16_t));
        value = static_cast<double>(temp);
        return true;
    }
    case sensor_msgs::PointField::INT16:
    {
        int16_t temp = 0;
        std::memcpy(&temp, data + field.offset, sizeof(int16_t));
        value = static_cast<double>(temp);
        return true;
    }
    case sensor_msgs::PointField::UINT32:
    {
        uint32_t temp = 0;
        std::memcpy(&temp, data + field.offset, sizeof(uint32_t));
        value = static_cast<double>(temp);
        return true;
    }
    case sensor_msgs::PointField::INT32:
    {
        int32_t temp = 0;
        std::memcpy(&temp, data + field.offset, sizeof(int32_t));
        value = static_cast<double>(temp);
        return true;
    }
    default:
        return false;
    }
}

int findField(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string &name)
{
    for (size_t i = 0; i < msg->fields.size(); ++i)
    {
        if (msg->fields[i].name == name)
            return static_cast<int>(i);
    }
    return -1;
}
} // namespace

namespace fastlio
{
    bool esti_plane(Eigen::Vector4d &out, const PointVector &points, const double &thresh)
    {
        Eigen::Matrix<double, NUM_MATCH_POINTS, 3> A;
        Eigen::Matrix<double, NUM_MATCH_POINTS, 1> b;
        A.setZero();
        b.setOnes();
        b *= -1.0;
        for (int i = 0; i < NUM_MATCH_POINTS; i++)
        {
            A(i, 0) = points[i].x;
            A(i, 1) = points[i].y;
            A(i, 2) = points[i].z;
        }

        Eigen::Vector3d normvec = A.colPivHouseholderQr().solve(b);

        double norm = normvec.norm();
        out[0] = normvec(0) / norm;
        out[1] = normvec(1) / norm;
        out[2] = normvec(2) / norm;
        out[3] = 1.0 / norm;

        for (int j = 0; j < NUM_MATCH_POINTS; j++)
        {
            if (std::fabs(out(0) * points[j].x + out(1) * points[j].y + out(2) * points[j].z + out(3)) > thresh)
            {
                return false;
            }
        }
        return true;
    }

    float sq_dist(const PointType &p1, const PointType &p2)
    {
        return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    }

    MTK::get_cov<process_noise_ikfom>::type process_noise_cov()
    {
        MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
        MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);   // 0.03
        MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na, 0.0001);   // *dt 0.01 0.01 * dt * dt 0.05
        MTK::setDiagonal<process_noise_ikfom, vect3, 6>(cov, &process_noise_ikfom::nbg, 0.00001); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
        MTK::setDiagonal<process_noise_ikfom, vect3, 9>(cov, &process_noise_ikfom::nba, 0.00001); // 0.001 0.05 0.0001/out 0.01
        return cov;
    }

    Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in)
    {
        Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
        vect3 omega;
        in.gyro.boxminus(omega, s.bg);
        vect3 a_inertial = s.rot * (in.acc - s.ba);
        for (int i = 0; i < 3; i++)
        {
            res(i) = s.vel[i];
            res(i + 3) = omega[i];
            res(i + 12) = a_inertial[i] + s.grav[i];
        }
        return res;
    }

    Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in)
    {
        Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
        cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
        vect3 acc_;
        in.acc.boxminus(acc_, s.ba);
        vect3 omega;
        in.gyro.boxminus(omega, s.bg);
        cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix() * MTK::hat(acc_);
        cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();
        Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
        Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
        s.S2_Mx(grav_matrix, vec, 21);
        cov.template block<3, 2>(12, 21) = grav_matrix;
        cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
        return cov;
    }

    Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in)
    {
        Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
        cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
        cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
        cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
        cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
        return cov;
    }

} // namespace name

void ImuData::callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mutex);
    double timestamp = msg->header.stamp.toSec();
    if (timestamp < last_timestamp)
    {
        ROS_WARN("imu loop back, clear buffer, last_timestamp: %f  current_timestamp: %f", last_timestamp, timestamp);
        buffer.clear();
    }
    last_timestamp = timestamp;
    buffer.emplace_back(timestamp,
                        msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z,
                        msg->angular_velocity.x,
                        msg->angular_velocity.y,
                        msg->angular_velocity.z);
}

void LivoxData::callback(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mutex);
    double timestamp = msg->header.stamp.toSec() - time_offset_lidar_to_imu;
    if (timestamp < last_timestamp)
    {
        ROS_WARN("livox loop back, clear buffer, last_timestamp: %f  current_timestamp: %f", last_timestamp, timestamp);
        buffer.clear();
        time_buffer.clear();
    }
    last_timestamp = timestamp;
    fastlio::PointCloudXYZI::Ptr ptr(new fastlio::PointCloudXYZI());
    livox2pcl(msg, ptr);
    // ROS_INFO("size: %lu",ptr->size());
    buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp);
}

void LivoxData::callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mutex);
    double timestamp = msg->header.stamp.toSec() - time_offset_lidar_to_imu;
    if (timestamp < last_timestamp)
    {
        ROS_WARN("pointcloud loop back, clear buffer, last_timestamp: %f  current_timestamp: %f", last_timestamp, timestamp);
        buffer.clear();
        time_buffer.clear();
        clearDualPending();
    }
    last_timestamp = timestamp;

    // The known A2 dual-JT128 topic contains two concatenated monotonic
    // timestamp runs.  Its rear run is commonly one message late and may be
    // repeated, so parsing both runs as if they belonged to the current header
    // silently corrupts IMU deskew.  Keep this special path behind the existing
    // explicit opt-in parameters so legacy timestamp normalization remains an
    // independent behavior for ordinary PointCloud2 users.
    if (use_point_timestamp && sync_dual_lidar_segments)
    {
        ++dual_input_frames;
        dual_last_input_wall = ros::WallTime::now();
        if (dual_enable_idle_flush && !dual_flush_timer_started)
        {
            ros::NodeHandle nh;
            dual_flush_timer = nh.createWallTimer(
                ros::WallDuration(0.2),
                &LivoxData::dualFlushTimerCallback, this);
            dual_flush_timer_started = true;
        }

        TimestampedCloudSegment front;
        TimestampedCloudSegment rear;
        if (!pointCloud2DualSegments(msg, front, rear))
        {
            ++dual_invalid_frames;
            // A malformed frame must not prevent older valid front scans from
            // aging out.  The header is still a useful monotonic watermark.
            if (!dual_has_newest_front || timestamp > dual_newest_front_time)
            {
                dual_newest_front_time = timestamp;
                dual_has_newest_front = true;
            }
            processDualPending(false);
            logDualStats(dual_input_frames == 1 || dual_input_frames % 100 == 0);
            return;
        }
        ++dual_valid_frames;

        const double tolerance = std::max(0.0, dual_sync_tolerance_sec);
        if (dual_has_newest_front &&
            front.begin <= dual_newest_front_time + tolerance)
        {
            // Header rollback is handled above.  Reaching this branch means a
            // duplicate/equal front scan, which must not produce repeated LIO
            // measurements.
            ++dual_invalid_frames;
            ROS_WARN_THROTTLE(2.0,
                              "Dual JT128 front scan is duplicate/out of order: newest=%.9f current=%.9f",
                              dual_newest_front_time, front.begin);
            processDualPending(false);
            logDualStats(dual_input_frames % 100 == 0);
            return;
        }

        dual_newest_front_time = front.begin;
        dual_has_newest_front = true;
        dual_front_pending.emplace_back(std::move(front));

        bool duplicate_rear = dual_has_last_emitted &&
                              rear.begin <= dual_last_emitted_time + tolerance;
        if (!duplicate_rear)
        {
            for (const auto &cached : dual_rear_pending)
            {
                if (std::fabs(cached.begin - rear.begin) <= tolerance)
                {
                    duplicate_rear = true;
                    break;
                }
            }
        }
        if (duplicate_rear)
        {
            ++dual_duplicate_rear_dropped;
        }
        else
        {
            auto insertion = std::upper_bound(
                dual_rear_pending.begin(), dual_rear_pending.end(), rear.begin,
                [](double stamp, const TimestampedCloudSegment &segment)
                { return stamp < segment.begin; });
            dual_rear_pending.insert(insertion, std::move(rear));
        }

        processDualPending(false);
        dual_max_front_pending_seen =
            std::max(dual_max_front_pending_seen, dual_front_pending.size());
        dual_max_rear_pending_seen =
            std::max(dual_max_rear_pending_seen, dual_rear_pending.size());
        logDualStats(dual_input_frames == 1 || dual_input_frames % 100 == 0);
        return;
    }

    fastlio::PointCloudXYZI::Ptr ptr(new fastlio::PointCloudXYZI());
    pointCloud2pcl(msg, ptr);
    if (ptr->empty())
    {
        ROS_WARN_THROTTLE(1.0, "PointCloud2 message converted to an empty cloud.");
        return;
    }
    buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp);
}

void LivoxData::livox2pcl(const livox_ros_driver2::CustomMsg::ConstPtr &msg, fastlio::PointCloudXYZI::Ptr &out)
{
    int point_num = msg->point_num;
    out->clear();
    out->reserve(point_num / filter_num + 1);
    uint valid_num = 0;
    for (uint i = 0; i < point_num; i++)
    {
        if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {
            valid_num++;
            if (valid_num % filter_num != 0)
                continue;
            fastlio::PointType p;
            p.x = msg->points[i].x;
            p.y = msg->points[i].y;
            p.z = msg->points[i].z;
            p.intensity = msg->points[i].reflectivity;
            p.curvature = msg->points[i].offset_time / float(1000000); // 纳秒->毫秒
            if ((p.x * p.x + p.y * p.y + p.z * p.z > (blind * blind)))
            {
                out->push_back(p);
            }
        }
    }
}

bool LivoxData::pointCloud2DualSegments(
    const sensor_msgs::PointCloud2::ConstPtr &msg,
    TimestampedCloudSegment &front,
    TimestampedCloudSegment &rear)
{
    const int x_idx = findField(msg, "x");
    const int y_idx = findField(msg, "y");
    const int z_idx = findField(msg, "z");
    const int intensity_idx = findField(msg, "intensity");
    const int timestamp_idx = findField(msg, timestamp_field);
    if (x_idx < 0 || y_idx < 0 || z_idx < 0 || timestamp_idx < 0 ||
        msg->point_step == 0 || msg->width == 0 || msg->height == 0)
    {
        ROS_WARN_THROTTLE(2.0,
                          "Dual JT128 PointCloud2 requires x/y/z and '%s' fields",
                          timestamp_field.c_str());
        return false;
    }
    const sensor_msgs::PointField &timestamp_desc = msg->fields[timestamp_idx];
    if (timestamp_desc.datatype != sensor_msgs::PointField::FLOAT64 ||
        timestamp_desc.count != 1 ||
        timestamp_desc.offset + sizeof(double) > msg->point_step)
    {
        ROS_WARN_THROTTLE(2.0,
                          "Dual JT128 '%s' must be one FLOAT64 within point_step",
                          timestamp_field.c_str());
        return false;
    }
    if (msg->is_bigendian)
    {
        ROS_WARN_THROTTLE(2.0,
                          "Big-endian dual JT128 PointCloud2 is unsupported");
        return false;
    }
    if (!std::isfinite(timestamp_scale_to_sec) || timestamp_scale_to_sec <= 0.0)
    {
        ROS_WARN_THROTTLE(2.0,
                          "Invalid pointcloud timestamp_scale_to_sec: %.9g",
                          timestamp_scale_to_sec);
        return false;
    }

    const size_t width = static_cast<size_t>(msg->width);
    const size_t height = static_cast<size_t>(msg->height);
    const size_t point_num = width * height;
    const size_t minimum_row_step = width * static_cast<size_t>(msg->point_step);
    if (msg->row_step < minimum_row_step)
        return false;
    const size_t required_bytes =
        (height - 1) * static_cast<size_t>(msg->row_step) + minimum_row_step;
    if (msg->data.size() < required_bytes)
        return false;

    auto pointData = [&](size_t index) -> const uint8_t *
    {
        const size_t row = index / width;
        const size_t column = index - row * width;
        return msg->data.data() + row * static_cast<size_t>(msg->row_step) +
               column * static_cast<size_t>(msg->point_step);
    };
    auto readTimestamp = [&](size_t index, double &stamp) -> bool
    {
        if (!fieldToDouble(pointData(index), timestamp_desc, stamp))
            return false;
        stamp *= timestamp_scale_to_sec;
        return std::isfinite(stamp);
    };

    double first_stamp = 0.0;
    if (!readTimestamp(0, first_stamp))
        return false;
    double previous_stamp = first_stamp;
    double front_end = first_stamp;
    double rear_begin = 0.0;
    double rear_end = 0.0;
    size_t split_index = point_num;
    int backward_jumps = 0;
    const double split_threshold =
        std::max(1.0e-3, std::max(0.0, timestamp_backward_jump_sec));
    constexpr double monotonic_tolerance_sec = 1.0e-6;

    for (size_t i = 1; i < point_num; ++i)
    {
        double stamp = 0.0;
        if (!readTimestamp(i, stamp))
            return false;
        if (stamp + monotonic_tolerance_sec < previous_stamp)
        {
            const double backward = previous_stamp - stamp;
            if (backward < split_threshold)
            {
                ROS_WARN_THROTTLE(2.0,
                                  "Dual JT128 timestamp jitter %.6f ms is not a valid segment boundary",
                                  backward * 1000.0);
                return false;
            }
            ++backward_jumps;
            if (backward_jumps > 1)
                return false;
            split_index = i;
            front_end = previous_stamp;
            rear_begin = stamp;
            rear_end = stamp;
        }
        else if (backward_jumps == 0)
        {
            front_end = std::max(front_end, stamp);
        }
        else
        {
            rear_end = std::max(rear_end, stamp);
        }
        previous_stamp = stamp;
    }

    if (backward_jumps != 1 || split_index == 0 || split_index >= point_num)
    {
        ROS_WARN_THROTTLE(2.0,
                          "Dual JT128 PointCloud2 expected exactly two timestamp runs, found %d boundaries",
                          backward_jumps);
        return false;
    }

    front.begin = first_stamp;
    front.end = front_end;
    rear.begin = rear_begin;
    rear.end = rear_end;
    const double front_duration = front.end - front.begin;
    const double rear_duration = rear.end - rear.begin;
    if (front_duration < dual_min_segment_duration_sec ||
        front_duration > dual_max_segment_duration_sec ||
        rear_duration < dual_min_segment_duration_sec ||
        rear_duration > dual_max_segment_duration_sec)
    {
        ROS_WARN_THROTTLE(2.0,
                          "Dual JT128 segment duration invalid: front=%.6f rear=%.6f s",
                          front_duration, rear_duration);
        return false;
    }

    const double header_stamp = msg->header.stamp.toSec();
    const double tolerance = std::max(0.0, dual_sync_tolerance_sec);
    if (!std::isfinite(header_stamp) ||
        std::fabs(header_stamp - front.begin) > tolerance)
    {
        ROS_WARN_THROTTLE(2.0,
                          "Dual JT128 header/front mismatch: header=%.9f front=%.9f",
                          header_stamp, front.begin);
        return false;
    }
    if (rear.begin < front.begin - std::max(dual_hold_sec, 0.0) - tolerance ||
        rear.begin > front.begin + tolerance)
    {
        ROS_WARN_THROTTLE(2.0,
                          "Dual JT128 rear start outside bounded sync window: front=%.9f rear=%.9f",
                          front.begin, rear.begin);
        return false;
    }

    front.cloud->clear();
    rear.cloud->clear();
    front.cloud->reserve(split_index / static_cast<size_t>(std::max(1, filter_num)) + 1);
    rear.cloud->reserve((point_num - split_index) /
                            static_cast<size_t>(std::max(1, filter_num)) +
                        1);
    const float blind_sq = static_cast<float>(blind * blind);

    auto convertSegment = [&](size_t begin_index, size_t end_index,
                              TimestampedCloudSegment &segment) -> bool
    {
        unsigned int valid_num = 0;
        for (size_t i = begin_index; i < end_index; ++i)
        {
            const uint8_t *data = pointData(i);
            float x = 0.0f, y = 0.0f, z = 0.0f, intensity = 0.0f;
            if (!fieldToFloat(data, msg->fields[x_idx], x) ||
                !fieldToFloat(data, msg->fields[y_idx], y) ||
                !fieldToFloat(data, msg->fields[z_idx], z))
                continue;
            if (intensity_idx >= 0)
                fieldToFloat(data, msg->fields[intensity_idx], intensity);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
                x * x + y * y + z * z <= blind_sq)
                continue;

            ++valid_num;
            if (filter_num > 1 && valid_num % filter_num != 0)
                continue;

            double stamp = 0.0;
            if (!readTimestamp(i, stamp))
                return false;
            const double relative_sec = stamp - segment.begin;
            if (relative_sec < -monotonic_tolerance_sec ||
                relative_sec > dual_max_segment_duration_sec + tolerance)
                return false;

            fastlio::PointType point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.intensity = intensity;
            point.curvature =
                static_cast<float>(std::max(0.0, relative_sec) * 1000.0);
            segment.cloud->push_back(point);
        }
        return !segment.cloud->empty();
    };

    if (!convertSegment(0, split_index, front) ||
        !convertSegment(split_index, point_num, rear))
        return false;

    // Both source runs are monotonic in this format, but sorting after spatial
    // filtering makes the invariant explicit for MeasureGroup::syncPackage,
    // which reads points.back() before IMUProcessor performs its own sort.
    auto byTime = [](const fastlio::PointType &lhs, const fastlio::PointType &rhs)
    { return lhs.curvature < rhs.curvature; };
    std::stable_sort(front.cloud->points.begin(), front.cloud->points.end(), byTime);
    std::stable_sort(rear.cloud->points.begin(), rear.cloud->points.end(), byTime);
    return true;
}

void LivoxData::emitDualScan(const TimestampedCloudSegment &front,
                             const TimestampedCloudSegment *rear)
{
    const double tolerance = std::max(0.0, dual_sync_tolerance_sec);
    const bool paired = rear != nullptr &&
                        std::fabs(front.begin - rear->begin) <= tolerance;
    // The bag header is exactly the front segment start.  Keep it as the
    // canonical scan epoch so consecutive LIO packages remain strictly ordered;
    // the sub-millisecond clock phase of a matched rear segment is represented
    // in each rear point's relative offset below.
    const double scan_begin = front.begin;
    if (dual_has_last_emitted && scan_begin <= dual_last_emitted_time)
    {
        ROS_WARN_THROTTLE(2.0,
                          "Dual JT128 output moved backwards: previous=%.9f current=%.9f",
                          dual_last_emitted_time, scan_begin);
        ++dual_invalid_frames;
        return;
    }

    fastlio::PointCloudXYZI::Ptr merged(new fastlio::PointCloudXYZI());
    const size_t rear_size = paired ? rear->cloud->size() : 0;
    merged->reserve(front.cloud->size() + rear_size);
    for (const auto &source : front.cloud->points)
        merged->push_back(source);
    if (paired)
    {
        const float rear_shift_ms =
            static_cast<float>((rear->begin - scan_begin) * 1000.0);
        for (const auto &source : rear->cloud->points)
        {
            fastlio::PointType point = source;
            point.curvature = std::max(0.0f, point.curvature + rear_shift_ms);
            merged->push_back(point);
        }
    }
    std::stable_sort(
        merged->points.begin(), merged->points.end(),
        [](const fastlio::PointType &lhs, const fastlio::PointType &rhs)
        { return lhs.curvature < rhs.curvature; });
    if (merged->empty() || !std::isfinite(merged->points.back().curvature))
    {
        ++dual_invalid_frames;
        return;
    }

    buffer.push_back(merged);
    time_buffer.push_back(scan_begin);
    const size_t max_output_frames =
        std::max<std::size_t>(1, max_output_buffer_frames);
    while (buffer.size() > max_output_frames)
    {
        buffer.pop_front();
        if (!time_buffer.empty())
            time_buffer.pop_front();
        ++dual_output_backlog_dropped;
        ROS_WARN_THROTTLE(2.0,
                          "Dual JT128 output backlog exceeded %zu frames; dropping oldest synchronized scan",
                          max_output_frames);
    }
    dual_last_emitted_time = scan_begin;
    dual_has_last_emitted = true;
    if (paired)
        ++dual_paired_frames;
    else
        ++dual_front_only_frames;
}

void LivoxData::processDualPending(bool force_all)
{
    const double tolerance = std::max(0.0, dual_sync_tolerance_sec);
    const double hold = std::max(0.0, dual_hold_sec);
    const size_t max_pending = std::max<std::size_t>(1, dual_max_pending);

    while (!dual_front_pending.empty())
    {
        const double front_stamp = dual_front_pending.front().begin;
        while (!dual_rear_pending.empty() &&
               dual_rear_pending.front().begin < front_stamp - tolerance)
        {
            dual_rear_pending.pop_front();
            ++dual_stale_rear_dropped;
        }

        auto best = dual_rear_pending.end();
        double best_error = tolerance;
        for (auto it = dual_rear_pending.begin();
             it != dual_rear_pending.end(); ++it)
        {
            const double error = std::fabs(it->begin - front_stamp);
            if (error <= best_error)
            {
                best_error = error;
                best = it;
            }
            if (it->begin > front_stamp + tolerance)
                break;
        }

        const bool expired_by_time =
            dual_has_newest_front &&
            dual_newest_front_time - front_stamp + tolerance >= hold;
        const bool expired_by_capacity =
            dual_front_pending.size() > max_pending;
        if (best == dual_rear_pending.end() && !force_all &&
            !expired_by_time && !expired_by_capacity)
            break;

        TimestampedCloudSegment front =
            std::move(dual_front_pending.front());
        dual_front_pending.pop_front();
        if (best != dual_rear_pending.end())
        {
            TimestampedCloudSegment rear = std::move(*best);
            dual_rear_pending.erase(best);
            emitDualScan(front, &rear);
        }
        else
        {
            emitDualScan(front, nullptr);
        }
    }

    while (dual_front_pending.size() > max_pending)
    {
        TimestampedCloudSegment front =
            std::move(dual_front_pending.front());
        dual_front_pending.pop_front();
        emitDualScan(front, nullptr);
    }
    while (dual_rear_pending.size() > max_pending)
    {
        dual_rear_pending.pop_front();
        ++dual_stale_rear_dropped;
    }
}

void LivoxData::clearDualPending()
{
    dual_front_pending.clear();
    dual_rear_pending.clear();
    dual_newest_front_time = 0.0;
    dual_last_emitted_time = 0.0;
    dual_has_newest_front = false;
    dual_has_last_emitted = false;
    dual_last_input_wall = ros::WallTime();
}

void LivoxData::logDualStats(bool force) const
{
    if (!force)
        return;
    ROS_INFO("Dual JT128 sync: input=%llu valid=%llu paired=%llu front_only=%llu "
             "duplicate_rear=%llu stale_rear=%llu output_backlog_drop=%llu invalid=%llu "
             "pending=%zu/%zu max_pending=%zu/%zu output_queue=%zu",
             static_cast<unsigned long long>(dual_input_frames),
             static_cast<unsigned long long>(dual_valid_frames),
             static_cast<unsigned long long>(dual_paired_frames),
             static_cast<unsigned long long>(dual_front_only_frames),
             static_cast<unsigned long long>(dual_duplicate_rear_dropped),
             static_cast<unsigned long long>(dual_stale_rear_dropped),
             static_cast<unsigned long long>(dual_output_backlog_dropped),
             static_cast<unsigned long long>(dual_invalid_frames),
             dual_front_pending.size(), dual_rear_pending.size(),
             dual_max_front_pending_seen, dual_max_rear_pending_seen,
             buffer.size());
}

void LivoxData::dualFlushTimerCallback(const ros::WallTimerEvent &)
{
    std::lock_guard<std::mutex> lock(mutex);
    if (!dual_enable_idle_flush)
        return;
    if (dual_front_pending.empty() || dual_last_input_wall.isZero())
        return;
    if ((ros::WallTime::now() - dual_last_input_wall).toSec() <
        std::max(0.1, dual_idle_flush_wall_sec))
        return;

    // rosbag playback may stop with up to dual_hold_sec of front scans still
    // pending.  Flush once after an idle interval so the final odometry sample
    // remains available for start/end closure measurement.
    processDualPending(true);
    logDualStats(true);
}

void LivoxData::pointCloud2pcl(const sensor_msgs::PointCloud2::ConstPtr &msg, fastlio::PointCloudXYZI::Ptr &out)
{
    out->clear();
    const int x_idx = findField(msg, "x");
    const int y_idx = findField(msg, "y");
    const int z_idx = findField(msg, "z");
    const int intensity_idx = findField(msg, "intensity");
    const int timestamp_idx = findField(msg, timestamp_field);
    if (x_idx < 0 || y_idx < 0 || z_idx < 0 || msg->point_step == 0)
    {
        ROS_WARN_THROTTLE(1.0, "PointCloud2 is missing x/y/z fields.");
        return;
    }

    const size_t point_num = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    if (point_num == 0 || msg->data.size() < point_num * msg->point_step)
        return;

    const float fallback_scan_period_ms =
        static_cast<float>(std::max(1.0e-3, fallback_scan_period_sec) * 1000.0);
    const float blind_sq = static_cast<float>(blind * blind);
    out->reserve(point_num / std::max(1, filter_num) + 1);
    std::vector<double> retained_timestamps;
    if (use_point_timestamp && timestamp_idx >= 0)
        retained_timestamps.reserve(point_num / std::max(1, filter_num) + 1);
    uint valid_num = 0;
    for (size_t i = 0; i < point_num; ++i)
    {
        const uint8_t *point_data = msg->data.data() + i * msg->point_step;
        float x = 0.0f, y = 0.0f, z = 0.0f, intensity = 0.0f;
        if (!fieldToFloat(point_data, msg->fields[x_idx], x) ||
            !fieldToFloat(point_data, msg->fields[y_idx], y) ||
            !fieldToFloat(point_data, msg->fields[z_idx], z))
        {
            continue;
        }
        if (intensity_idx >= 0)
            fieldToFloat(point_data, msg->fields[intensity_idx], intensity);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            continue;
        if (x * x + y * y + z * z <= blind_sq)
            continue;

        valid_num++;
        if (filter_num > 1 && valid_num % filter_num != 0)
            continue;

        fastlio::PointType p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = intensity;
        p.curvature = 0.0f;
        out->push_back(p);

        if (use_point_timestamp && timestamp_idx >= 0)
        {
            double point_time = std::numeric_limits<double>::quiet_NaN();
            if (fieldToDouble(point_data, msg->fields[timestamp_idx], point_time))
                point_time *= timestamp_scale_to_sec;
            retained_timestamps.push_back(point_time);
        }
    }

    bool applied_point_timing = false;
    int segment_count = 0;
    if (use_point_timestamp && normalize_timestamp_segments &&
        timestamp_idx >= 0 && retained_timestamps.size() == out->size() &&
        !retained_timestamps.empty())
    {
        std::vector<int> segment_ids(retained_timestamps.size(), 0);
        std::vector<double> segment_starts;
        std::vector<double> segment_ends;
        segment_starts.push_back(retained_timestamps.front());
        segment_ends.push_back(retained_timestamps.front());
        segment_count = 1;
        bool timestamps_valid = std::isfinite(retained_timestamps.front());
        double previous_time = retained_timestamps.front();

        for (size_t i = 1; i < retained_timestamps.size(); ++i)
        {
            const double point_time = retained_timestamps[i];
            if (!std::isfinite(point_time))
            {
                timestamps_valid = false;
                break;
            }
            if (point_time < previous_time - std::max(0.0, timestamp_backward_jump_sec))
            {
                ++segment_count;
                if (segment_count > std::max(1, max_timestamp_segments))
                {
                    timestamps_valid = false;
                    break;
                }
                segment_starts.push_back(point_time);
                segment_ends.push_back(point_time);
            }
            segment_ids[i] = segment_count - 1;
            segment_ends.back() = std::max(segment_ends.back(), point_time);
            previous_time = point_time;
        }

        if (timestamps_valid)
        {
            for (size_t i = 0; i < out->size(); ++i)
            {
                const int segment = segment_ids[i];
                const double relative_sec =
                    std::max(0.0, retained_timestamps[i] - segment_starts[segment]);
                out->points[i].curvature = static_cast<float>(relative_sec * 1000.0);
            }
            std::stable_sort(out->points.begin(), out->points.end(),
                             [](const fastlio::PointType &lhs, const fastlio::PointType &rhs)
                             { return lhs.curvature < rhs.curvature; });
            applied_point_timing = true;
            double maximum_duration = 0.0;
            for (size_t segment = 0; segment < segment_starts.size(); ++segment)
                maximum_duration = std::max(maximum_duration,
                                            segment_ends[segment] - segment_starts[segment]);
            ROS_INFO_ONCE("PointCloud2 timing: field=%s, normalized_segments=%d, max_duration=%.6f s",
                          timestamp_field.c_str(), segment_count, maximum_duration);
        }
    }

    if (!applied_point_timing)
    {
        const size_t retained_num = out->size();
        for (size_t i = 0; i < retained_num; ++i)
        {
            out->points[i].curvature = retained_num > 1
                ? fallback_scan_period_ms * static_cast<float>(i) /
                      static_cast<float>(retained_num - 1)
                : 0.0f;
        }
        if (use_point_timestamp)
        {
            ROS_WARN_THROTTLE(5.0,
                              "PointCloud2 timestamp timing unavailable or invalid; using %.3f ms fallback",
                              fallback_scan_period_ms);
        }
    }
}

bool MeasureGroup::syncPackage(ImuData &imu_data, LivoxData &livox_data)
{
    // ROS callbacks run on AsyncSpinner threads while mapping consumes and
    // removes samples here. Lock both queues for the complete transaction so
    // deque reads/pops cannot race with callback push/clear operations.
    std::scoped_lock<std::mutex, std::mutex> lock(imu_data.mutex, livox_data.mutex);

    // A bounded dual-lidar output queue may discard an old scan while this
    // object is waiting for its ending IMU sample.  Never pop a newer queue
    // entry on behalf of that discarded cached pointer.
    if (lidar_pushed &&
        (livox_data.buffer.empty() || livox_data.time_buffer.empty() ||
         livox_data.buffer.front() != lidar ||
         std::fabs(livox_data.time_buffer.front() - lidar_time_begin) > 1.0e-9))
    {
        lidar_pushed = false;
        lidar.reset();
    }

    if (lidar_pushed && livox_data.last_timestamp + 1e-3 < lidar_time_begin)
    {
        lidar_pushed = false;
        lidar.reset();
    }

    while (!livox_data.buffer.empty() &&
           (!livox_data.buffer.front() || livox_data.buffer.front()->empty()))
    {
        livox_data.buffer.pop_front();
        if (!livox_data.time_buffer.empty())
            livox_data.time_buffer.pop_front();
    }
    if (imu_data.buffer.empty() || livox_data.buffer.empty() ||
        livox_data.time_buffer.empty())
        return false;

    if (!lidar_pushed)
    {
        lidar = livox_data.buffer.front();
        lidar_time_begin = livox_data.time_buffer.front();
        lidar_time_end = lidar_time_begin + lidar->points.back().curvature / double(1000);
        lidar_pushed = true;
    }

    if (imu_data.last_timestamp < lidar_time_end)
        return false;
    double imu_time = imu_data.buffer.front().timestamp;
    imus.clear();
    while (!imu_data.buffer.empty() && (imu_time < lidar_time_end))
    {
        imu_time = imu_data.buffer.front().timestamp;
        if (imu_time > lidar_time_end)
            break;
        imus.push_back(imu_data.buffer.front());
        imu_data.buffer.pop_front();
    }
    livox_data.buffer.pop_front();
    livox_data.time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

sensor_msgs::PointCloud2 pcl2msg(fastlio::PointCloudXYZI::Ptr inp, std::string &frame_id, const double &timestamp)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*inp, msg);
    if (timestamp < 0)
        msg.header.stamp = ros::Time().now();
    else
        msg.header.stamp = ros::Time().fromSec(timestamp);
    msg.header.frame_id = frame_id;
    return msg;
}

nav_msgs::Odometry eigen2Odometry(const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos, const std::string &frame_id, const std::string &child_frame_id, const double &timestamp)
{
    nav_msgs::Odometry odom;
    odom.header.frame_id = frame_id;
    odom.header.stamp = ros::Time().fromSec(timestamp);
    odom.child_frame_id = child_frame_id;
    Eigen::Quaterniond q = Eigen::Quaterniond(rot);
    odom.pose.pose.position.x = pos(0);
    odom.pose.pose.position.y = pos(1);
    odom.pose.pose.position.z = pos(2);

    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    return odom;
}

geometry_msgs::TransformStamped eigen2Transform(const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos, const std::string &frame_id, const std::string &child_frame_id, const double &timestamp)
{
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = frame_id;
    transform.header.stamp = ros::Time().fromSec(timestamp);
    transform.child_frame_id = child_frame_id;
    transform.transform.translation.x = pos(0);
    transform.transform.translation.y = pos(1);
    transform.transform.translation.z = pos(2);
    Eigen::Quaterniond q = Eigen::Quaterniond(rot);
    // std::cout << rot << std::endl;
    // std::cout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
    transform.transform.rotation.w = q.w();
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    return transform;
}

Eigen::Vector3d rotate2rpy(Eigen::Matrix3d &rot)
{
    double roll = std::atan2(rot(2, 1), rot(2, 2));
    double pitch = asin(-rot(2, 0));
    double yaw = std::atan2(rot(1, 0), rot(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw);
}
