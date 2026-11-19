// Sparse visualization must exactly match the authoritative collision buffers,
// including overlapping inflation, partial resets and sliding-ring reuse.
#include <plan_env/grid_map.h>
#include <chrono>
#include <iostream>
#include <set>
#include <array>
#include <stdexcept>

using Keys=std::set<std::array<int,3>>;
std::array<int,3> key(GridMap& grid,const Eigen::Vector3d& p) {
  Eigen::Vector3i id;grid.posToIndex(p,id);return {{id.x(),id.y(),id.z()}};
}
void check(GridMap& grid,const char* stage) {
  Eigen::Vector3d origin,size;grid.getRegion(origin,size);
  const double r=grid.getResolution();const Eigen::Vector3i n=(size/r).array().round().cast<int>();
  Keys dense[2],sparse[2];
  for(int x=0;x<n.x();++x)for(int y=0;y<n.y();++y)for(int z=0;z<n.z();++z) {
    const Eigen::Vector3d p=origin+r*Eigen::Vector3d(x+.5,y+.5,z+.5);
    if(grid.getOccupancy(p)==1)dense[0].insert(key(grid,p));
    if(grid.getObstacleOccupancy(p,0.)==1)dense[1].insert(key(grid,p));
  }
  for(int layer=0;layer<2;++layer) {
    const auto points=grid.occupiedVoxelCenters(layer,false);
    for(const auto& p:points)sparse[layer].insert(key(grid,p));
    if(sparse[layer]!=dense[layer]||sparse[layer].size()!=points.size())
      throw std::runtime_error(std::string(stage)+": sparse/dense mismatch layer "+std::to_string(layer));
  }
  std::cout<<"PASS "<<stage<<" raw="<<dense[0].size()<<" inflated="<<dense[1].size()<<"\n";
}
void moveSensor(ros::Publisher& pub,double x,double y) {
  nav_msgs::Odometry pose;pose.header.stamp=ros::Time::now();pose.pose.pose.orientation.w=1.;
  pose.pose.pose.position.x=x;pose.pose.pose.position.y=y;
  pub.publish(pose);
  for(int i=0;i<10;++i){ros::WallDuration(.01).sleep();ros::spinOnce();}
}
int main(int argc,char** argv) {
  ros::init(argc,argv,"map_publication_test");ros::NodeHandle nh("/scan_planner_node");
  nh.setParam("grid_map/require_plan_free",false);nh.setParam("grid_map/resolution",.05);
  nh.setParam("grid_map/double_cylinder_offset",0.);nh.setParam("grid_map/need_extrinsic",false);
  nh.setParam("grid_map/ground_height",-1.);nh.setParam("grid_map/map_sliding_en",true);
  for(const auto& axis:{"x","y","z"})nh.setParam(std::string("grid_map/sliding_map_size_")+axis,2.);
  try {
    {
      GridMap grid;grid.initMap(nh);check(grid,"empty");
      grid.setOccupied({.1,.1,.4});grid.setOccupied({.15,.1,.4});grid.setOccupied({-.85,-.1,.4});
      check(grid,"overlapping inflation");grid.setOccupancy({.1,.1,.4},0.);check(grid,"clear one overlap");
      grid.resetBuffer({.12,.07,.37},{.18,.13,.43});check(grid,"partial reset");
      grid.setOccupied({.2,.2,.4});
      auto sensor=nh.advertise<nav_msgs::Odometry>("/grid_map/sensor_pose",1);
      for(int i=0;i<100&&sensor.getNumSubscribers()==0;++i){ros::spinOnce();ros::WallDuration(.01).sleep();}
      if(!sensor.getNumSubscribers())throw std::runtime_error("sensor test connection missing");
      moveSensor(sensor,.4,0.);check(grid,"sliding positive");
      if(grid.getOccupancy(Eigen::Vector3d(.2,.2,.4))!=1)throw std::runtime_error("retained cell lost");
      for(int i=0;i<7;++i){grid.setOccupied({.4+i*.2,.2,.4});moveSensor(sensor,.6+i*.2,-.2);check(grid,"ring reuse");}
      moveSensor(sensor,-.4,.3);check(grid,"sliding negative/diagonal");
      moveSensor(sensor,5.,0.);check(grid,"large jump reset");
      grid.setOccupied({5.,0.,.4});grid.resetBuffer();check(grid,"full reset");
    }
    // Real production dimensions: four million cells, sparse obstacle returns.
    nh.setParam("grid_map/sliding_map_size_x",10.);nh.setParam("grid_map/sliding_map_size_y",10.);
    nh.setParam("grid_map/sliding_map_size_z",5.);nh.setParam("grid_map/ground_height",-2.5);
    GridMap grid;grid.initMap(nh);
    for(int i=0;i<3000;++i)grid.setOccupied({-.975+.05*(i%40),-.975+.05*((i/40)%40),.025+.05*(i/1600)});
    check(grid,"production dimensions");
    const auto start=std::chrono::steady_clock::now();size_t count=0;
    for(int i=0;i<100;++i)count+=grid.occupiedVoxelCenters(false,false).size();
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/100.;
    std::cout<<"BENCH raw sparse collection "<<ms<<" ms, "<<count/100<<" occupied / 4000000 total cells\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
