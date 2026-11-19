#include <plan_env/plan_free_space.hpp>
#include <plan_manage/reference_ingress.hpp>
#include <cassert>

int main() {
  RobotFootprint body;PlanFreeSpace map;std::vector<Eigen::Vector3d> cells;
  for (int x=-2;x<14;++x) for (int y=-7;y<7;++y)
    if (y>=3 || (x>=4 && x<9)) cells.emplace_back((x+.5)*.2,(y+.5)*.2,-.5);
  map.set(cells,.2,body,.3);
  const std::vector<Eigen::Vector3d> route{{2.3,1.1,-.1},{1.9,.9,-.1},{1.3,.7,-.1},{1.3,-1.,-.1}};
  const double yaw=-3.141592653589793;
  auto free=[&](const Eigen::Vector3d& p,double h) { return map.contains(p,h,.4); };
  assert(free(route.front(),yaw));
  assert(!free(route.front(),std::atan2(-.2,-.4)));
  const auto adapted=scan_planner::referenceIngress(route,yaw,free);
  assert(adapted.size()>route.size());
  assert((adapted.front()-route.front()).norm()<1e-9);
  assert((adapted.back()-route.back()).norm()<1e-9);
  for (size_t i=1;i<adapted.size();++i) {
    const auto delta=(adapted[i]-adapted[i-1]).eval();
    if (delta.head<2>().norm()>1e-6)
      assert(free(adapted[i-1],std::atan2(delta.y(),delta.x())));
    assert(std::abs(adapted[i].z()+.1)<1e-9);
  }
  // Occupied starts and wholly blocked alternatives cannot produce a bypass.
  assert(scan_planner::referenceIngress(route,yaw,[](const Eigen::Vector3d&,double){return false;})==route);
}
