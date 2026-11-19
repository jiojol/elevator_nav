#include <plan_manage/go2_tracking_policy.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

using scan_planner::Go2TrackingPolicy;

int main()
{
  Go2TrackingPolicy policy;
  auto previous = policy.step(0.0, {0.35, 0.0}, 0.2, 0., 3., 0., .01);
  for (int i = 0; i < 200; ++i)
  {
    auto cmd = policy.step(0.0, {0.35, 0.0}, 0.2, 0., 3., 0., .01);
    assert(cmd.lateral == 0. && cmd.forward >= 0. && cmd.forward <= .20);
    assert(std::abs(cmd.forward - previous.forward) <= .0025 + 1e-12);
    assert(std::abs(cmd.yaw_rate - previous.yaw_rate) <= .008 + 1e-12);
    previous = cmd;
  }
  assert(previous.forward > .199 && previous.yaw_rate == 0.);

  // A sudden corner stops translation for turning; orientation noise at
  // the entry threshold cannot repeatedly toggle translation on and off.
  auto cmd = policy.step(1.0, {.2, .3}, .2, 0., 3., 0., .01);
  assert(cmd.rotating && cmd.forward == 0. && cmd.yaw_rate > 0.);
  for (int i = 0; i < 120; ++i)
  {
    cmd = policy.step(1.0, {.2, .3}, .2, 0., 3., (i % 2 ? .36 : .34), .01);
    assert(cmd.rotating && cmd.lateral == 0.);
  }
  assert(cmd.forward == 0.);
  cmd = policy.step(1.0, {.2, .3}, .2, 0., 3., .9, .01);
  assert(!cmd.rotating && cmd.forward > 0.);

  // A goal behind the body rotates in place from rest; no backwards/sideways
  // translation is emitted. Reset is the immediate stop/new-goal operation.
  policy.reset();
  for (int i = 0; i < 100; ++i)
  {
    cmd = policy.step(3.141592653589793, {-.35, 0.}, .2, 0., 3., 0., .01);
    assert(cmd.rotating && cmd.forward == 0. && cmd.lateral == 0.);
    assert(std::abs(cmd.yaw_rate) <= .4 + 1e-12);
  }

  // Left and right tracking errors are corrected by yaw with zero strafing.
  for (double sign : {-1., 1.})
  {
    policy.reset();
    for (int i = 0; i < 100; ++i)
      cmd = policy.step(sign * .3, {.3, sign * .1}, .2, 0., 2., 0., .01);
    assert(cmd.forward > 0. && cmd.yaw_rate * sign > 0. && cmd.lateral == 0.);
  }

  // Zero desired speed near the end must stay zero rather than activating
  // the minimum progress speed. Braking and a slower-than-plan body stay finite.
  policy.reset();
  cmd = policy.step(0., {.02, 0.}, 0., 0., .02, 0., .01);
  assert(cmd.forward == 0.);
  for (int i = 0; i < 200; ++i)
    cmd = policy.step(0., {.35, 0.}, .05, .20, 3., 0., .01);
  assert(cmd.forward > .19 && cmd.lateral == 0.);
  policy.reset();
  policy.limits.max_forward = .8;
  policy.limits.rotate_enter = .95;
  for (int i = 0; i < 200; ++i)
  {
    cmd = policy.step(.75, {.35*std::cos(.75), .35*std::sin(.75)}, .8, 0., 3., 0., .01);
    assert(!cmd.rotating && cmd.forward > 0. && cmd.lateral == 0.);
    assert(std::abs(cmd.yaw_rate) <= .4 + 1e-12);
  }
  // Recorded fault: a 0.331 m/s turn limit was followed by 0.397 m/s output.
  // Build speed first, then abruptly tighten curvature without entering spin.
  policy.reset();policy.limits.max_forward=.8;policy.limits.max_yaw=.5;
  for (int i=0;i<500;++i) policy.step(0.,{.65,0.},.8,0.,5.,0.,.01);
  assert(policy.command().forward>.79);
  for (int i=0;i<100;++i) {
    const auto turn=policy.step(.50,{.35,.20},.8,0.,3.,0.,.01);
    assert(turn.forward<=turn.turn_limit+1e-12);
    assert(turn.forward<=turn.brake_limit+1e-12);
  }
  // Steering must ramp toward the requested arc independently of the
  // temporarily capped forward output, otherwise this pair locks at a crawl.
  assert(policy.command().forward>.15);
  // Quantized heading changes around a straight corridor must not force a
  // stop just because the smoothed yaw command still has the previous sign.
  policy.reset();
  for (int i=0;i<400;++i) policy.step(0.,{.6,0.},.8,0.,5.,0.,.01);
  for (int i=0;i<100;++i) {
    const double error=i%2?.01:-.01;
    const auto straight=policy.step(error,{.6,.6*std::tan(error)},.8,0.,5.,0.,.01);
    assert(straight.forward>.75);
    assert(straight.forward<=straight.turn_limit+1e-12);
  }
  std::cout << "PASS: speed/acceleration limits, forward-only steering, hysteresis, reset and endpoint behavior\n";
}
