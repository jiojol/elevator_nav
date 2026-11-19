#include <plan_manage/reference_timing.hpp>
#include <cassert>
#include <iostream>
int main()
{
  using namespace scan_planner;
  const auto straight = timeReference({{0,0,.4},{3,0,.4}}, .8, .25, 0., 0.);
  const double duration = straight.interval * (straight.points.size() - 1);
  assert(duration > 6. && duration < 10.);
  double max_speed = 0.;
  for (size_t i=1; i<straight.points.size(); ++i)
    max_speed = std::max(max_speed, (straight.points[i]-straight.points[i-1]).norm()/straight.interval);
  assert(max_speed > .6 && max_speed <= .8);
  assert((straight.points.front()-Eigen::Vector3d(0,0,.4)).norm()<1e-9);
  assert((straight.points.back()-Eigen::Vector3d(3,0,.4)).norm()<1e-9);
  const auto corner = timeReference({{0,0,.4},{1.5,0,.4},{1.5,1.5,.4}}, .8, .25, 0., 0.);
  assert(corner.interval*(corner.points.size()-1) > duration);
  const auto responsive = timeReference({{0,0,.4},{1.5,0,.4},{1.5,1.5,.4}}, .8, .5, 0., 0., .8);
  assert(responsive.interval*(responsive.points.size()-1) < .8*corner.interval*(corner.points.size()-1));
  ReferencePath route;
  assert(route.set({{-1.4,8.9,-.54},{-1.9,8.9,-.53},{-1.9,8.3,-.52}}));
  assert(!route.roundsCorner(0.,.9,{-1.4704,8.3483,-.5436},{-1.4826,8.3482,-.5426}));
  assert(route.roundsCorner(.1,.9,{-1.75,8.66,-.53},{-1.76,8.65,-.53}));
  std::cout<<"PASS acceleration-aware reference timing: 3 m duration="<<duration
           <<" s, peak sample speed="<<max_speed<<" m/s; corner is slower; deep shortcut rejected\n";
}
