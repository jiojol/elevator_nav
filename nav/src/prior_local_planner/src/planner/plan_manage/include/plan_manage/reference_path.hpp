#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace scan_planner
{
// Arc length belongs to the accepted route, not to elapsed trajectory time.
class ReferencePath
{
public:
  struct Projection
  {
    double arc = 0.0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    double xy_error = std::numeric_limits<double>::infinity();
    double z_error = std::numeric_limits<double>::infinity();
  };

  bool set(const std::vector<Eigen::Vector3d> &points)
  {
    clear();
    for (const auto &point : points)
    {
      if (!point.allFinite()) { clear(); return false; }
      if (points_.empty() || (point - points_.back()).norm() > 1e-6)
      {
        arc_.push_back(points_.empty() ? 0.0 : arc_.back() + (point - points_.back()).norm());
        points_.push_back(point);
      }
    }
    if (points_.size() < 2) { clear(); return false; }
    return true;
  }

  void clear() { points_.clear(); arc_.clear(); }
  bool empty() const { return points_.empty(); }
  double length() const { return empty() ? 0.0 : arc_.back(); }

  Eigen::Vector3d at(double distance) const
  {
    distance = std::max(0.0, std::min(length(), distance));
    const auto upper = std::upper_bound(arc_.begin(), arc_.end(), distance);
    if (upper == arc_.end()) return points_.back();
    const size_t next = static_cast<size_t>(upper - arc_.begin());
    const double fraction = (distance - arc_[next - 1]) / (arc_[next] - arc_[next - 1]);
    return points_[next - 1] + fraction * (points_[next] - points_[next - 1]);
  }

  // Preserve global-route terrain breakpoints in the spline initializer.
  std::vector<Eigen::Vector3d> vertices(double low,double high) const
  {
    if (empty()) return {};
    low=std::max(0.,low);high=std::min(length(),high);
    if (high<=low) return {};
    std::vector<Eigen::Vector3d> result{at(low)};
    for (size_t i=0;i<arc_.size();++i)
      if (arc_[i]>low && arc_[i]<high) result.push_back(points_[i]);
    result.push_back(at(high));
    return result;
  }

  Eigen::Vector3d tangent(double distance) const
  {
    const double before = std::max(0.0, distance - 0.05);
    const double after = std::min(length(), distance + 0.05);
    Eigen::Vector3d delta = at(after) - at(before);
    if (delta.norm() <= 1e-9) return Eigen::Vector3d::Zero();
    return delta.normalized();
  }

  Projection project(const Eigen::Vector3d &position, double low, double high, bool planar = false) const
  {
    Projection best;
    double score = std::numeric_limits<double>::infinity();
    low = std::max(0.0, low);
    high = std::min(length(), high);
    for (size_t i = 1; i < points_.size(); ++i)
    {
      if (arc_[i] < low || arc_[i - 1] > high) continue;
      const Eigen::Vector3d delta = points_[i] - points_[i - 1];
      const double span = arc_[i] - arc_[i - 1];
      const double lower = std::max(0.0, (low - arc_[i - 1]) / span);
      const double upper = std::min(1.0, (high - arc_[i - 1]) / span);
      // Kinematic terrain owns Z independently of this smoothed curve. Its
      // tracking progress uses XY; full 3D collision queries remain separate.
      const double denominator=planar?delta.head<2>().squaredNorm():delta.squaredNorm();
      const double numerator=planar?(position-points_[i-1]).head<2>().dot(delta.head<2>()):(position-points_[i-1]).dot(delta);
      const double fraction = denominator>1e-12?std::max(lower,std::min(upper,numerator/denominator)):lower;
      const Eigen::Vector3d point = points_[i - 1] + fraction * delta;
      const Eigen::Vector3d error = position - point;
      const double candidate = error.head<2>().squaredNorm() + (planar?0.:4.0 * error.z() * error.z());
      if (candidate < score)
      {
        score = candidate;
        best.arc = arc_[i - 1] + fraction * span;
        best.point = point;
        best.xy_error = error.head<2>().norm();
        best.z_error = std::abs(error.z());
      }
    }
    return best;
  }

private:
  std::vector<Eigen::Vector3d> points_;
  std::vector<double> arc_;
};
} // namespace scan_planner
