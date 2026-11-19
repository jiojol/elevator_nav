#include <plan_manage/local_target.hpp>
#include <plan_manage/start_alignment.hpp>
#include <cassert>
#include <iostream>
int main(){
 using Eigen::Vector3d;using namespace scan_planner;
 const Vector3d start(.522,5.549,2.964),goal(.45,5.55,3.133);
 ReferencePath route;assert(route.set({start,goal}));
 auto clear=[](const Vector3d&,double){return true;};
 assert(localTargetArc(route,start,0.,3.5,.3,.05,clear)<0.);
 assert(localTargetArc(route,start,0.,3.5,.3,.05,clear,true)>0.);
 assert(localTargetArc(route,start,0.,3.5,.3,.05,[](const Vector3d&,double){return false;},true)<0.);
 assert(!chooseStartAlignment(start,0.,goal,clear).required);
 const auto turn=chooseStartAlignment(start,0.,goal,clear,2.,.2,.025);
 assert(turn.required&&turn.allowed);
 const auto blocked=chooseStartAlignment(start,0.,goal,[](const Vector3d&,double){return false;},2.,.2,.025);
 assert(blocked.required&&!blocked.allowed);
 std::cout<<"PASS short terrain terminal keeps legacy behavior and collision guards\n";
}
