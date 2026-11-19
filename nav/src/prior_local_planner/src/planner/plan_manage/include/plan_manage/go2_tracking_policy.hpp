#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace scan_planner
{
struct Go2TrackingLimits
{
  double max_forward = 0.20, max_yaw = 0.40;
  double linear_accel = 0.25, angular_accel = 0.80;
  double rotate_enter = 0.65, rotate_exit = 0.25;
  double start_align_enter = 0.25, start_align_exit = 0.10, start_align_min_yaw = 0.40;
  double yaw_gain = 1.5, position_gain = 0.8;
  double heading_filter_time = 0.20, min_forward = 0.04, finish_distance = 0.15;
};

struct Go2TrackingCommand
{
  double forward = 0.0, lateral = 0.0, yaw_rate = 0.0, heading_error = 0.0;
  bool rotating = false;
  double requested_forward = 0.0, turn_limit = 0.0, brake_limit = 0.0, curvature = 0.0;
};

class Go2TrackingPolicy
{
public:
  Go2TrackingLimits limits;
  const Go2TrackingCommand &command() const { return command_; }

  static double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
  static double bound(double value, double limit) { return std::max(-limit, std::min(limit, value)); }
  static double slew(double current, double target, double step)
  { return current + bound(target - current, step); }

  void reset()
  {
    command_ = Go2TrackingCommand();
    heading_ready_ = false;
    heading_ = 0.0;
    align_from_rest_ = false;
    rest_aligning_ = false;
  }

  void alignBeforeNextMotion() { align_from_rest_=true; }

  Go2TrackingCommand step(double desired_heading, const Eigen::Vector2d &target_delta,
                          double feedforward, double along_error, double remaining,
                          double yaw, double dt, double terminal_speed = 0.0)
  {
    dt = std::max(0.0, std::min(0.05, dt));
    const double raw_error = wrap(desired_heading - yaw);
    if (!heading_ready_)
    {
      heading_ = desired_heading;
      heading_ready_ = true;
    }
    else
    {
      const double alpha = dt / std::max(1e-9, limits.heading_filter_time + dt);
      heading_ = wrap(heading_ + alpha * wrap(desired_heading - heading_));
    }
    const double error = wrap(heading_ - yaw);
    // Explicitly requested only when resuming from an obstacle hold.
    // Ordinary starts and rolling handoffs retain normal arc tracking.
    if (align_from_rest_) {
      rest_aligning_=std::abs(raw_error)>limits.start_align_enter;
      command_.rotating=rest_aligning_;
      align_from_rest_=false;
    }
    const double exit_threshold=rest_aligning_?limits.start_align_exit:limits.rotate_exit;
    if (!command_.rotating && std::abs(raw_error) >= limits.rotate_enter)
      command_.rotating = true;
    else if (command_.rotating && std::abs(raw_error) <= exit_threshold
             && std::abs(error) <= exit_threshold) {
      command_.rotating = false;
      rest_aligning_ = false;
    }

    const double curvature = 2.0 * std::sin(error) / std::max(0.20, target_delta.norm());
    const double raw_curvature = 2.0 * std::sin(raw_error) / std::max(0.20, target_delta.norm());
    const double turn_curvature = std::max(std::abs(curvature), std::abs(raw_curvature));
    command_.curvature = curvature;
    command_.turn_limit = turn_curvature > 1e-6 ? limits.max_yaw / turn_curvature : limits.max_forward;
    terminal_speed = std::max(0.0, std::min(limits.max_forward, terminal_speed));
    command_.brake_limit = std::sqrt(terminal_speed * terminal_speed + 2.0 * limits.linear_accel * std::max(0.0, remaining));
    command_.requested_forward = 0.0;
    double forward = 0.0;
    if (!command_.rotating)
    {
      forward = std::max(0.0, feedforward + limits.position_gain * along_error);
      if (remaining > limits.finish_distance + 0.10 && along_error > -0.05)
        forward = std::max(forward, limits.min_forward);
      forward = std::min(limits.max_forward, forward);
      command_.requested_forward = forward;
      forward = std::min(forward, command_.turn_limit);
      forward = std::min(forward, command_.brake_limit);
    }
    command_.forward = std::max(0.0, slew(command_.forward, forward, limits.linear_accel * dt));
    // Slewing must never override a newly tighter geometric/braking limit.
    command_.forward=std::min(command_.forward,command_.rotating?0.:
        std::min(limits.max_forward,std::min(command_.turn_limit,command_.brake_limit)));
    double yaw_rate = command_.rotating ? bound(limits.yaw_gain * error, limits.max_yaw)
                                        : bound(forward * curvature, limits.max_yaw);
    if (command_.rotating && rest_aligning_ && std::abs(error)>limits.start_align_exit) {
      const double minimum=std::min(limits.max_yaw,limits.start_align_min_yaw);
      yaw_rate=std::copysign(std::max(std::abs(yaw_rate),minimum),error);
    }
    command_.yaw_rate = slew(command_.yaw_rate, yaw_rate, limits.angular_accel * dt);
    command_.lateral = 0.0;
    command_.heading_error = error;
    return command_;
  }

private:
  Go2TrackingCommand command_;
  bool heading_ready_ = false;
  bool align_from_rest_ = false, rest_aligning_ = false;
  double heading_ = 0.0;
};
} // namespace scan_planner
