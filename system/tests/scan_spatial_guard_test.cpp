#include <plan_manage/planner_manager.h>
#include <ros/ros.h>
#include <iostream>

int main(int argc,char** argv) {
  ros::init(argc,argv,"scan_spatial_guard_test");
  ros::NodeHandle nh("/scan_planner_node");
  scan_planner::SCANPlannerManager manager;
  manager.grid_map_.reset(new GridMap);
  nh.setParam("grid_map/require_plan_free",false);
  manager.grid_map_->initMap(nh);
  Eigen::MatrixXd controls(3,9);
  for (int i=0;i<9;++i) controls.col(i)=Eigen::Vector3d(-2.4+.6*i,0.,.4);
  scan_planner::UniformBspline curve(controls,3,1.);
  if (!manager.checkSpatialFeasibility(curve)) {
    std::cerr<<"clear trajectory unexpectedly rejected\n";return 1;
  }
  const double duration=curve.getTimeSum();
  const auto obstacle=curve.evaluateDeBoorT(.94*duration);
  manager.grid_map_->setOccupied(obstacle);
  for (double t=0.;t<duration*2./3.;t+=.01) {
    if (manager.grid_map_->getInflateOccupancy(curve.evaluateDeBoorT(t),0.)!=0) {
      std::cerr<<"fixture obstacle is not confined to terminal horizon\n";return 2;
    }
  }
  if (manager.checkSpatialFeasibility(curve)) {
    std::cerr<<"terminal obstacle was accepted\n";return 3;
  }
  std::cout<<"PASS: clear curve accepted; occupied terminal horizon rejected\n";
  return 0;
}
