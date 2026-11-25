#pragma once

#include <bspline_opt/uniform_bspline.h>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace scan_upstream_planner
{
struct TrajectoryDynamicBounds
{
  double horizontal_velocity = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  bool valid = false;
};

// A quadratic velocity's squared norm is quartic. Isolate the stationary
// points on [0,1] by splitting its cubic derivative at its quadratic roots.
// This avoids treating the derivative control polygon as the actual peak.
inline double quadraticSpeedMaximum(const Eigen::Vector3d &a,
                                     const Eigen::Vector3d &b,
                                     const Eigen::Vector3d &c)
{
  const std::array<double, 4> g = {b.dot(c), b.squaredNorm() + 2.0 * a.dot(c),
                                  3.0 * a.dot(b), 2.0 * a.squaredNorm()};
  const auto value = [&](double u) { return ((g[3] * u + g[2]) * u + g[1]) * u + g[0]; };
  const auto squared = [&](double u) { return (c + u * (b + u * a)).squaredNorm(); };
  std::vector<double> partitions = {0.0, 1.0};
  const auto insert = [&](double u) {
    if (std::isfinite(u) && u > 0.0 && u < 1.0)
      partitions.push_back(u);
  };
  const double A = 3.0 * g[3], B = 2.0 * g[2], C = g[1];
  const double epsilon = 1e-13 * std::max({1.0, std::abs(A), std::abs(B), std::abs(C)});
  if (std::abs(A) <= epsilon)
  {
    if (std::abs(B) > epsilon)
      insert(-C / B);
  }
  else
  {
    const double discriminant = B * B - 4.0 * A * C;
    if (discriminant >= 0.0)
    {
      const double root = std::sqrt(discriminant);
      const double q = -0.5 * (B + std::copysign(root, B));
      insert(q / A);
      if (std::abs(q) > epsilon)
        insert(C / q);
    }
  }
  std::sort(partitions.begin(), partitions.end());
  double maximum = std::max(squared(0.0), squared(1.0));
  for (size_t i = 1; i < partitions.size(); ++i)
  {
    double lo = partitions[i - 1], hi = partitions[i];
    double left = value(lo), right = value(hi);
    maximum = std::max({maximum, squared(lo), squared(hi)});
    if ((left < 0.0 && right > 0.0) || (left > 0.0 && right < 0.0))
    {
      for (int step = 0; step < 40; ++step)
      {
        const double mid = (lo + hi) * 0.5, v = value(mid);
        if ((left < 0.0 && v < 0.0) || (left > 0.0 && v > 0.0))
          lo = mid;
        else
          hi = mid;
      }
      maximum = std::max(maximum, squared((lo + hi) * 0.5));
    }
  }
  return std::sqrt(std::max(0.0, maximum));
}

inline TrajectoryDynamicBounds cubicDynamicBounds(UniformBspline position)
{
  TrajectoryDynamicBounds result;
  const Eigen::VectorXd knots = position.getKnot();
  const Eigen::MatrixXd control = position.getControlPoint();
  if (control.rows() != 3 || control.cols() < 4 || knots.size() != control.cols() + 4 ||
      !control.allFinite() || !knots.allFinite() || !std::is_sorted(knots.data(), knots.data() + knots.size()))
    return result;
  const double start = knots(3), duration = knots(control.cols()) - start;
  if (!std::isfinite(duration) || duration <= 0.0)
    return result;
  auto velocity = position.getDerivative();
  auto acceleration = velocity.getDerivative();
  if (!velocity.getControlPoint().allFinite() || !acceleration.getControlPoint().allFinite())
    return result;
  for (Eigen::Index i = 3; i < control.cols(); ++i)
  {
    const double lo = knots(i) - start, hi = knots(i + 1) - start, dt = hi - lo;
    if (dt <= 0.0)
      continue;
    const Eigen::Vector3d c = velocity.evaluateDeBoorT(lo);
    const Eigen::Vector3d first_acc = acceleration.evaluateDeBoorT(lo);
    const Eigen::Vector3d last_acc = acceleration.evaluateDeBoorT(hi);
    const Eigen::Vector3d b = first_acc * dt;
    const Eigen::Vector3d a = 0.5 * dt * (last_acc - first_acc);
    result.velocity = std::max(result.velocity, quadraticSpeedMaximum(a, b, c));
    result.horizontal_velocity = std::max(result.horizontal_velocity,
        quadraticSpeedMaximum(Eigen::Vector3d(a.x(), a.y(), 0.0),
                              Eigen::Vector3d(b.x(), b.y(), 0.0),
                              Eigen::Vector3d(c.x(), c.y(), 0.0)));
    // A cubic spline's acceleration is linear in each span. Its norm is
    // convex, so its maximum is at a span endpoint, including internal knots.
    result.acceleration = std::max({result.acceleration, first_acc.norm(), last_acc.norm()});
  }
  result.valid = std::isfinite(result.horizontal_velocity) && std::isfinite(result.velocity) &&
                 std::isfinite(result.acceleration);
  return result;
}

inline double requiredTimeScale(const TrajectoryDynamicBounds &bounds,
                                double horizontal_limit, double velocity_limit, double acceleration_limit)
{
  if (!bounds.valid || horizontal_limit <= 0.0 || velocity_limit <= 0.0 || acceleration_limit <= 0.0)
    return std::numeric_limits<double>::infinity();
  return std::max({1.0, bounds.horizontal_velocity / horizontal_limit,
                  bounds.velocity / velocity_limit, std::sqrt(bounds.acceleration / acceleration_limit)});
}
} // namespace scan_upstream_planner
