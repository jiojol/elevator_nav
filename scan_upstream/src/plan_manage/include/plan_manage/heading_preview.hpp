#pragma once

#include <algorithm>
#include <bspline_opt/uniform_bspline.h>

namespace scan_upstream_planner
{
inline Eigen::Vector2d headingPreview(UniformBspline &trajectory, double time, double duration,
                                    double lookahead, const Eigen::Vector2d &position)
{
  constexpr double minimum_displacement = 0.06;
  double sample_time = std::min(duration, time + std::max(0.0, lookahead));
  const double limit = std::min(duration, sample_time + 1.0);
  while (true)
  {
    const Eigen::Vector2d displacement = trajectory.evaluateDeBoorT(sample_time).head<2>() - position;
    if (displacement.squaredNorm() >= minimum_displacement * minimum_displacement)
      return displacement;
    if (sample_time >= limit)
      return Eigen::Vector2d::Zero();
    sample_time = std::min(limit, sample_time + 0.15);
  }
}
}
