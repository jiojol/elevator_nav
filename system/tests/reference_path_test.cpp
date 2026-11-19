#include <plan_manage/reference_path.hpp>
#include <cassert>
#include <iostream>

using scan_planner::ReferencePath;
using Eigen::Vector3d;

int main()
{
  ReferencePath route;
  assert(route.set({{0, 0, .4}, {1, 0, .4}, {1, 1, 1.4}, {0, 1, 1.4}}));
  assert((route.at(1.) - Vector3d(1, 0, .4)).norm() < 1e-9);
  assert((route.at(1. + std::sqrt(2.)) - Vector3d(1, 1, 1.4)).norm() < 1e-9);
  const auto samples = route.sample(0., route.length(), .1, {0, 0, .4});
  assert(samples.size() >= 7 && (samples.back() - Vector3d(0, 1, 1.4)).norm() < 1e-9);
  double progress = 0.;
  for (const auto &point : samples)
  {
    const auto projected = route.project(point, progress, route.length());
    assert(projected.xy_error < 1e-8 && projected.z_error < 1e-8);
    assert(projected.arc + 1e-8 >= progress);
    progress = projected.arc;
  }

  assert(route.set({{0, 0, .4}, {1, 0, .4}, {1, .2, .4}, {0, .2, .4}}));
  auto projected = route.project({.05, .2, .4}, 0., .5);
  assert(projected.arc < .1 && projected.xy_error > .19);
  projected = route.project({.05, .2, .4}, 1.5, route.length());
  assert(projected.arc > 2. && projected.xy_error < 1e-9);

  assert(route.set({{0, 0, .4}, {1, 0, 1.4}, {2, 0, .4}}));
  const auto peak = route.at(std::sqrt(2.));
  assert(std::abs(peak.z() - 1.4) < 1e-9);
  const Vector3d actual_start(0., .1, .55);
  const auto adjusted = route.sample(0., route.length(), .1, actual_start);
  assert((adjusted.front() - actual_start).norm() < 1e-9);
  assert((adjusted.back() - Vector3d(2., 0., .4)).norm() < 1e-9);
  assert(route.set({{0, 0, .4}, {0, 0, .4}, {.1, 0, .4}}));
  assert(route.sample(0., .1, .2, {0, 0, .4}).size() == 7);
  assert(!route.set({{0, 0, .4}, {NAN, 0, .4}}));
  assert(route.empty());
  assert(route.set({{0, 0, .4}, {5, 0, .4}}));
  auto clear = [](double) { return true; };
  assert(std::abs(route.localTarget(0., 2., {0, 0, .4}, clear) - 2.) < 1e-9);
  // Only the first ten centimetres and the route beyond the nominal horizon
  // are clear: the old backward search selected an unplannable 0.10 m segment.
  auto obstructed = [](double arc) { return arc < .15 || arc > 2.4; };
  assert(route.localTarget(0., 2., {0, 0, .4}, obstructed) > 2.4);
  assert(route.localTarget(0., 2., {0, 0, .4}, [](double arc) { return arc < .15; }) < 0.);
  assert(route.localTarget(0., 2., {0, 0, .4}, [](double) { return false; }) < 0.);
  const double backed = route.localTarget(0., 2., {0, 0, .4}, [](double arc) { return arc < 1.; });
  assert(backed >= .2 && backed < 1.);
  std::cout << "Reference geometry, ordering, height and blocked/degenerate endpoint checks PASS\n";
}
