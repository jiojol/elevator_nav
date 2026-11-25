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

struct TurnPreview
{
  double longest_turn = 0.0;
  double phase = 0.0;
  bool clear = true;
};

// Two seconds of controller wall time. During a pure turn yaw advances while
// the spline clock is frozen, just as in closed_loop_controller::cmdCallback.
template<typename FootprintClear>
TurnPreview previewTurns(UniformBspline &curve, double yaw, double lookahead,
                         double threshold, double kp, double max_rate, FootprintClear footprint_clear)
{
  constexpr double dt = 0.02;
  const double duration = curve.getTimeSum();
  double run = 0.0;
  TurnPreview result;
  for (int i = 0; i < 100 && result.phase < duration; ++i)
  {
    const Eigen::Vector3d p = curve.evaluateDeBoorT(result.phase);
    const Eigen::Vector2d d = headingPreview(curve, result.phase, duration, lookahead, p.head<2>());
    const double error = d.isZero(1e-9) ? 0.0 : std::remainder(std::atan2(d.y(), d.x()) - yaw, 2.0 * M_PI);
    const bool turning = std::abs(error) > threshold;
    run = turning ? run + dt : 0.0;
    result.longest_turn = std::max(result.longest_turn, run);
    if (!turning)
      result.phase = std::min(result.phase + dt, duration);
    yaw += std::clamp(kp * error, -max_rate, max_rate) * dt;
    if (!footprint_clear(curve.evaluateDeBoorT(result.phase), yaw))
    {
      result.clear = false;
      break;
    }
  }
  return result;
}
}
