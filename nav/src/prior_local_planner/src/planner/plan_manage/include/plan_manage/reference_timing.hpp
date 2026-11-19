#pragma once
#include <plan_manage/reference_path.hpp>

namespace scan_planner
{
struct TimedReference
{
  std::vector<Eigen::Vector3d> points;
  double interval = 0.0, end_speed = 0.0;
};

// Sample a spatial route at equal time intervals. Small spatial steps at the
// ends/turns and larger steps on straights let a uniform B-spline accelerate
// without stretching every straight interval to the slowest starting step.
inline TimedReference timeReference(const std::vector<Eigen::Vector3d> &points,
                                    double max_speed, double max_accel,
                                    double start_speed, double end_speed,
                                    double max_yaw = 0.40, double sample_interval = 0.30)
{
  TimedReference result;
  ReferencePath route;
  if (!route.set(points) || max_speed <= 0.0 || max_accel <= 0.0
      || !std::isfinite(sample_interval) || sample_interval<=0.) return result;
  const int count = std::max(3, static_cast<int>(std::ceil(route.length() / 0.04)) + 1);
  const double ds = route.length() / (count - 1), acceleration = 0.8 * max_accel;
  std::vector<double> speed(count, 0.95 * max_speed), time(count, 0.0);
  for (int i = 0; i < count; ++i)
  {
    const double s = ds * i;
    const double lo = std::max(0.0, s - 0.35), hi = std::min(route.length(), s + 0.35);
    const Eigen::Vector3d a = route.at(s) - route.at(lo), b = route.at(hi) - route.at(s);
    if (a.head<2>().norm() > 0.02 && b.head<2>().norm() > 0.02)
    {
      const double angle = std::abs(std::atan2(a.x() * b.y() - a.y() * b.x(), a.head<2>().dot(b.head<2>())));
      const double curvature = angle / std::max(0.10, 0.5 * (hi - lo));
      if (curvature > 1e-6)
        speed[i] = std::min(speed[i], std::min(0.9 * max_yaw / curvature, std::sqrt(acceleration / curvature)));
    }
  }
  speed.front() = std::min(speed.front(), std::max(0.0, start_speed));
  speed.back() = std::min(speed.back(), std::max(0.0, end_speed));
  for (int i = 1; i < count; ++i)
    speed[i] = std::min(speed[i], std::sqrt(speed[i - 1] * speed[i - 1] + 2.0 * acceleration * ds));
  for (int i = count - 2; i >= 0; --i)
    speed[i] = std::min(speed[i], std::sqrt(speed[i + 1] * speed[i + 1] + 2.0 * acceleration * ds));
  for (int i = 1; i < count; ++i)
    time[i] = time[i - 1] + 2.0 * ds / std::max(1e-6, speed[i - 1] + speed[i]);
  const int samples = std::max(7, static_cast<int>(std::ceil(time.back() / sample_interval)) + 1);
  result.interval = time.back() / (samples - 1);
  result.end_speed = speed.back();
  for (int i = 0; i < samples; ++i)
  {
    const double t = i * result.interval;
    auto upper = std::upper_bound(time.begin(), time.end(), t);
    const int j = std::min(count - 1, std::max(1, static_cast<int>(upper - time.begin())));
    const double tau = std::max(0.0, t - time[j - 1]), duration = time[j] - time[j - 1];
    const double accel = (speed[j] - speed[j - 1]) / duration;
    const double arc = ds * (j - 1) + speed[j - 1] * tau + 0.5 * accel * tau * tau;
    result.points.push_back(route.at(std::min(route.length(), arc)));
  }
  result.points.front() = points.front();
  result.points.back() = points.back();
  return result;
}
} // namespace scan_planner
