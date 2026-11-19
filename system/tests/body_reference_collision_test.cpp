#include <plan_env/grid_map.h>
#include <dog_nav_bringup/planar_reference.hpp>
#include <ros/ros.h>
#include <iostream>

int main(int argc,char** argv) {
  ros::init(argc,argv,"body_reference_collision_test");
  ros::NodeHandle nh("/scan_planner_node");
  nh.setParam("grid_map/require_plan_free",false);
  nh.setParam("grid_map/ground_height",-2.5);
  nh.setParam("grid_map/map_sliding_en",false);
  GridMap grid;grid.initMap(nh);
  // Latest recording at 1793252061.6. Translate world Y by -9 m (180
  // map voxels) to fit the private test map without changing occupancy math.
  const Eigen::Vector3d sensor(.013842753,.225358162,-.511188073);
  const Eigen::Vector2d offset(-.221454,-.018929);
  const double measured_yaw=1.821680150,departure_yaw=-.413;
  // Actual recorded occupied voxel centres, not an invented obstacle.
  grid.setOccupied({-.475,.575,-.425});
  grid.setOccupied({-.325,.575,-.425});
  const auto false_centre=dog_nav::bodyReference(sensor,departure_yaw,offset);
  const auto measured_centre=dog_nav::bodyReference(sensor,measured_yaw,offset);
  if (grid.getObstacleOccupancy(false_centre,departure_yaw)!=1) {
    std::cerr<<"recorded false collision was not reproduced\n";return 1;
  }
  if (grid.getObstacleOccupancy(measured_centre,departure_yaw)!=0 ||
      grid.getObstacleOccupancy(measured_centre,measured_yaw)!=0) {
    std::cerr<<"measured body centre still blocked\n";return 2;
  }
  std::cout<<"PASS: recorded new-goal false collision reproduced; fixed body centre stays clear\n";
}
