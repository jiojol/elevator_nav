#include <plan_manage/reference_path.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
int main(){
 scan_planner::ReferencePath p;
 assert(p.set({{0,0,1.6},{.5,0,1.6},{.8,0,1.0},{1.2,0,.8}}));
 Eigen::Vector3d measured(.8,0,1.6);
 const auto old=p.project(measured,0.,p.length());
 const auto actual=p.project(measured,0.,p.length(),true);
 assert(old.point.x()<.7);
 assert(std::abs(actual.point.x()-.8)<1e-9);
 assert(actual.z_error>.5); // Diagnostics retain the real vertical difference.
 const auto old_target=p.at(old.arc+.10),new_target=p.at(actual.arc+.10);
 assert(old_target.x()<measured.x());assert(new_target.x()>measured.x());
 const auto on_curve=p.project({.8,0,1.0},0.,p.length());
 assert(on_curve.xy_error<1e-9&&on_curve.z_error<1e-9);
 assert(p.set({{0,0,.4},{1,0,.4},{1,.2,.4},{0,.2,.4}}));
 const auto bounded=p.project({.05,.2,9.},0.,.5,true);
 assert(bounded.arc<.1&&bounded.xy_error>.19);
 std::cout<<"PASS: stair height cannot move kinematic tracking target behind chassis; 3D default and bounded progress retained\n";
}
