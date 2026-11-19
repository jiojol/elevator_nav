#include <plan_manage/planner_manager.h>
#include <plan_manage/start_alignment.hpp>
#include <plan_manage/local_target.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <boost/make_shared.hpp>
#include <fstream>
#include <iostream>

std::vector<Eigen::Vector3d> readPoints(const std::string& path) {
  std::ifstream file(path);std::vector<Eigen::Vector3d> points;double x,y,z;
  while (file>>x>>y>>z) points.emplace_back(x,y,z);
  if (points.empty()) throw std::runtime_error("empty fixture: "+path);
  return points;
}
int main(int argc,char** argv) {
  ros::init(argc,argv,"narrow_prior_replan_test");
  if (argc<3) return 2;
  const std::string folder=argv[1],mode=argv[2];
  ros::NodeHandle nh("/scan_planner_node");
  nh.setParam("grid_map/ground_height",-2.5);nh.setParam("grid_map/map_sliding_en",false);
  scan_planner::SCANPlannerManager manager;
  manager.initPlanModules(nh,scan_planner::PlanningVisualization::Ptr(new scan_planner::PlanningVisualization(nh)));
  const auto prior=readPoints(folder+"/prior.xyz");
  auto cloud=boost::make_shared<sensor_msgs::PointCloud2>();cloud->header.frame_id="map";
  sensor_msgs::PointCloud2Modifier modifier(*cloud);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(prior.size());
  sensor_msgs::PointCloud2Iterator<float> x(*cloud,"x"),y(*cloud,"y"),z(*cloud,"z");
  for (const auto& p:prior) { *x=p.x();*y=p.y();*z=p.z();++x;++y;++z; }
  manager.grid_map_->planFreeCallback(cloud);
  for (const auto& point:readPoints(folder+"/occupancy.xyz")) manager.grid_map_->setOccupied(point);
  std::ifstream file(folder+"/state.txt");Eigen::Vector3d start,velocity;double qx,qy,qz,qw;
  file>>start.x()>>start.y()>>start.z()>>velocity.x()>>velocity.y()>>velocity.z()>>qx>>qy>>qz>>qw;
  velocity=Eigen::Quaterniond(qw,qx,qy,qz).normalized()*velocity;
  scan_planner::ReferencePath route;
  if (!route.set(readPoints(folder+"/route_"+mode+".xyz"))) return 3;
  const Eigen::Quaterniond body_rotation(qw,qx,qy,qz);
  const auto heading=body_rotation.normalized().toRotationMatrix().col(0).eval();
  double body_yaw=std::atan2(heading.y(),heading.x());
  if(mode=="aligned"||mode=="aligned_stop"){
    const Eigen::Vector3d aim=route.at(std::min(route.length(),.65))-start;
    body_yaw=std::atan2(aim.y(),aim.x());velocity.setZero();
  }
  if (mode!="recorded") manager.measured_start_yaw_=body_yaw;
  std::cout<<"RECORDED_BODY_SUPPORT prior="<<manager.grid_map_->isPlanFree(start,body_yaw)
           <<" live_occupancy="<<manager.grid_map_->getObstacleOccupancy(start,body_yaw)<<" yaw="<<body_yaw<<"\n";
  for (double a=0.;a<=route.length();a+=.2) {
    const auto point=route.at(a),direction=route.tangent(a);
    const double h=std::atan2(direction.y(),direction.x());
    std::cout<<"ROUTE_QUERY "<<a<<" p="<<point.transpose()<<" prior="<<manager.grid_map_->isPlanFree(point,h)
             <<" live="<<manager.grid_map_->getObstacleOccupancy(point,h)<<"\n";
  }
  const auto alignment=scan_planner::chooseStartAlignment(start,body_yaw,route.at(std::min(route.length(),.65)),[&](const Eigen::Vector3d& p,double h){return manager.grid_map_->getInflateOccupancy(p,h)==0;});
  std::cout<<"RECORDED_START_ALIGNMENT required="<<alignment.required<<" allowed="<<alignment.allowed<<" angle="<<alignment.signed_angle<<" target_yaw="<<alignment.target_yaw<<"\n";
  const auto final_tangent=route.tangent(route.length());
  const double final_yaw=std::atan2(final_tangent.y(),final_tangent.x());
  std::cout<<"RECORDED_GOAL_SUPPORT prior="<<manager.grid_map_->isPlanFree(route.at(route.length()),final_yaw)
           <<" live_occupancy="<<manager.grid_map_->getObstacleOccupancy(route.at(route.length()),final_yaw)<<"\n";
  const double progress=route.project(start,0.,.5).arc;
  double horizon=3.5;nh.param("fsm/planning_horizon",horizon,3.5);
  const double arc=scan_planner::localTargetArc(route,start,progress,horizon,manager.pp_.ctrl_pt_dist,
      manager.grid_map_->getResolution(),[&](const Eigen::Vector3d& p,double h) {
        return manager.grid_map_->getInflateOccupancy(p,h)==0;
      });
  if(arc<0.) { std::cerr<<"no supported local target\n";return 4; }
  const auto target=route.at(arc);Eigen::Vector3d target_velocity=route.tangent(arc)*manager.pp_.max_vel_;
  if (route.length()-arc<manager.pp_.max_vel_*manager.pp_.max_vel_/(2.*manager.pp_.max_acc_)) target_velocity.setZero();
  if(mode=="aligned_stop")target_velocity.setZero();
  manager.local_reference_=route.vertices(progress,arc);manager.local_reference_.front()=start;
  AStar diagnostic;diagnostic.initGridMap(manager.grid_map_,Eigen::Vector3i(100,100,100));
  const auto tangent=route.tangent(arc);const double target_yaw=std::atan2(tangent.y(),tangent.x());
  const auto search_result=diagnostic.AstarSearch(manager.grid_map_->getResolution(),start,target,body_yaw,target_yaw);
  std::cout<<"RECORDED_DIRECT_ASTAR status="<<int(search_result)<<" reason="<<diagnostic.lastFailure()<<"\n";
  if(search_result==ASTAR_RET::SUCCESS){std::ofstream path(folder+"/astar_"+mode+".xyz");for(const auto& p:diagnostic.getPath())path<<p.transpose()<<"\n";}
  const bool ok=manager.reboundReplan(start,velocity,Eigen::Vector3d::Zero(),target,target_velocity,true,false);
  if (ok) {
    if((manager.local_data_.position_traj_.evaluateDeBoorT(0.)-start).norm()>1e-6)
      throw std::runtime_error("published curve start shifted away from measured input");
    std::ofstream curve(folder+"/trajectory_"+mode+".xyz");
    auto spline=manager.local_data_.position_traj_;
    if(mode=="actual" && velocity.norm()<1e-6 && manager.grid_map_->terrainEnabled()) {
      const auto moving=spline.getDerivative().evaluateDeBoorT(std::min(.001,spline.getTimeSum()*.001));
      const double delta=std::atan2(std::sin(std::atan2(moving.y(),moving.x())-body_yaw),
                                    std::cos(std::atan2(moving.y(),moving.x())-body_yaw));
      if(moving.head<2>().norm()<1e-12 || std::abs(delta)>.05)
        throw std::runtime_error("stationary curve startup heading differs from measured body");
      std::cout<<"RECORDED_START_HEADING_ERROR "<<delta<<"\n";
    }
    for (int i=0;i<=300;++i) curve<<spline.evaluateDeBoorT(spline.getTimeSum()*i/300.).transpose()<<"\n";
  }
  std::cout<<"RECORDED_NARROW_REPLAN mode="<<mode<<" success="<<ok<<" target="<<target.transpose()<<" seed_points="<<manager.local_reference_.size()<<"\n";
  return ok?0:1;
}
