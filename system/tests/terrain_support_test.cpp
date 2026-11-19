#include <plan_env/plan_free_space.hpp>
#include <cassert>
#include <iostream>
std::vector<Eigen::Vector3d> stairs(bool hole=false,bool cliff=false) {
  std::vector<Eigen::Vector3d> p;
  for(int x=-10;x<=10;++x)for(int y=-10;y<=10;++y) {
    if(hole&&x==1&&y==1)continue;
    double z=.18*std::floor(double(x)/2.);
    if(cliff&&x>=1)z+=1.;
    p.emplace_back((x+.5)*.1,(y+.5)*.1,z);
  }
  return p;
}
int main(){
 RobotFootprint r;r.length=.8;r.width=.46;r.margin=.05;
 PlanFreeSpace map;Eigen::Vector3d body(.05,.05,.4);
 map.set(stairs(),.1,r,.3);assert(!map.contains(body,0.,.4));
 map.set(stairs(),.1,r,.3,.22);assert(map.contains(body,0.,.4));
 assert(map.contains(body,1.57,.4));
 assert(!map.contains(body+Eigen::Vector3d(0,0,1.),0.,.4));
 double height;assert(map.terrainHeight(.25,.05,0.,height));assert(std::abs(height-.18)<1e-9);
 map.set(stairs(true),.1,r,.3,.22);assert(!map.contains(body,0.,.4));
 map.set(stairs(false,true),.1,r,.3,.22);assert(!map.contains(body,0.,.4));
 auto flat=stairs();for(auto& p:flat)p.z()=0.;map.set(flat,.1,r,.3);assert(map.contains(body,0.,.4));
 assert(!map.contains(Eigen::Vector3d(1.02,.05,.4),0.,.4));
 std::cout<<"PASS terrain: connected stairs, heading, height, hole, cliff and legacy flat guards\n";
}
