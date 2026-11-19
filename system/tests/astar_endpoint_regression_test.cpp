#include <bspline_opt/bspline_optimizer.h>
#include <plan_manage/start_alignment.hpp>
#include <plan_manage/local_target.hpp>
#include <plan_manage/collision_progress.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <boost/make_shared.hpp>
#include <iostream>
#include <stdexcept>
using Eigen::Vector3d;
void check(bool ok,const char* name){if(!ok)throw std::runtime_error(name);std::cout<<"PASS "<<name<<"\n";}
void prior(GridMap& grid,bool elbow=true){
  std::vector<Vector3d> points;
  for(int x=-12;x<=5;++x)for(int y=-3;y<=12;++y){
    const double px=(x+.5)*.2,py=(y+.5)*.2;
    if(elbow?((px<.8&&std::abs(py)<.4)||(std::abs(px)<.4&&py>=0&&py<2.)):(std::abs(py)<.4))points.emplace_back(px,py,0.);
  }
  auto cloud=boost::make_shared<sensor_msgs::PointCloud2>();cloud->header.frame_id="map";
  sensor_msgs::PointCloud2Modifier mod(*cloud);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float>x(*cloud,"x"),y(*cloud,"y"),z(*cloud,"z");
  for(const auto& p:points){*x=p.x();*y=p.y();*z=p.z();++x;++y;++z;}grid.planFreeCallback(cloud);
}
int main(int argc,char** argv){
 scan_planner::ControlPoints gradient_controls;gradient_controls.resize(6);
 gradient_controls.flag_temp[2]=true;
 check(!gradient_controls.propagateDirections(0,5,2),"empty gradient flags cannot become propagation anchors");
 gradient_controls.flag_temp[3]=true;gradient_controls.base_point[3].push_back({1,0,0});gradient_controls.direction[3].push_back({0,1,0});
 check(gradient_controls.propagateDirections(0,5,2)&&!gradient_controls.base_point[2].empty()&&
       gradient_controls.direction[0].back().allFinite(),"valid rebound direction repairs empty flagged neighbors safely");
 const std::vector<Vector3d> uneven{{0,0,.4},{0,.7,.4},{.15,1.1,.4},{.5,1.3,.4},{1.,1.5,.4},{2.,1.5,.4},{2.3,1.7,.4}};
 const std::vector<Vector3d> boundaries{{0,.8,0},{.8,0,0},{.1,.2,0},{-.1,0,0}};
 Eigen::MatrixXd fitted;scan_planner::UniformBspline::parameterizeToBspline(.3,uneven,boundaries,fitted);
 scan_planner::UniformBspline fit(fitted,3,.3);auto fit_v=fit.getDerivative(),fit_a=fit_v.getDerivative();
 check((fit.evaluateDeBoorT(0.)-uneven.front()).norm()<1e-9&&
       (fit.evaluateDeBoorT(fit.getTimeSum())-uneven.back()).norm()<1e-9,"reference fitting preserves both endpoint positions exactly");
 check((fit_v.evaluateDeBoorT(0.)-boundaries[0]).norm()<1e-9&&
       (fit_v.evaluateDeBoorT(fit.getTimeSum())-boundaries[1]).norm()<1e-9,"reference fitting preserves both endpoint velocities exactly");
 check((fit_a.evaluateDeBoorT(0.)-boundaries[2]).norm()<1e-9&&
       (fit_a.evaluateDeBoorT(fit.getTimeSum())-boundaries[3]).norm()<1e-9,"reference fitting preserves both endpoint accelerations exactly");
 Eigen::MatrixXd straight_controls(3,9);for(int i=0;i<9;++i)straight_controls.col(i)=Vector3d(-.5+.5*i,0,.4);
 scan_planner::UniformBspline moving(straight_controls,3,.5);scan_planner::CollisionProgress progress;
 progress.reset(moving,.05);const double measured_time=progress.projectTime({.2,0,.4});
 check(std::abs(measured_time-.2)<1e-6,"collision window follows measured body progress instead of wall time");
 check(scan_planner::collisionStopDistance(.8,.5,.1,.05,1.)>1.4,"braking allowance adds physical stopping distance to the reaction horizon");
 const Vector3d centre(0.,0.,.4),behind(-1.,0.,.4);
 const auto clear=scan_planner::chooseStartAlignment(centre,0.,behind,[](const Vector3d&,double){return true;});
 check(clear.required&&clear.allowed&&std::abs(std::abs(clear.signed_angle)-M_PI)<1e-9,"reverse route admits a fully checked stationary action");
 const auto denied=scan_planner::chooseStartAlignment(centre,0.,behind,[](const Vector3d&,double){return false;});
 check(denied.required&&!denied.allowed,"blocked turn cannot receive a rotation permit");
 const auto alternative=scan_planner::chooseStartAlignment(centre,0.,behind,[](const Vector3d&,double h){return std::abs(scan_planner::alignmentWrap(h-1.))>.1;});
 check(alternative.allowed&&alternative.signed_angle<0.,"blocked short turn uses only a separately checked complementary arc");
 const auto forward=scan_planner::chooseStartAlignment(centre,0.,Vector3d(1.,0.,.4),[](const Vector3d&,double){return false;});
 check(!forward.required,"forward starts retain normal trajectory planning");
 check(scan_planner::alignmentRemaining(M_PI,-.01,M_PI)>M_PI,"selected reverse direction survives the pi boundary");
 scan_planner::ReferencePath straight;straight.set({{0.,0.,.4},{6.,0.,.4}});
 const auto always_free=[](const Vector3d&,double){return true;};
 check(std::abs(scan_planner::localTargetArc(straight,centre,0.,3.5,.3,.05,always_free)-3.5)<1e-9,"clear nominal local target is unchanged");
 const auto gaps=[](const Vector3d& p,double){return !(p.x()>3.25&&p.x()<3.55)&&!(p.x()>3.85&&p.x()<4.15);};
 const double selected=scan_planner::localTargetArc(straight,centre,0.,3.5,.3,.05,gaps);
 check(selected>=4.45&&selected<=6.,"isolated free endpoint is not preferred over a supported terminal neighborhood");
 check(scan_planner::localTargetArc(straight,centre,0.,3.5,.3,.05,[](const Vector3d&,double){return false;})<0.,"blocked route does not manufacture an endpoint");
 ros::init(argc,argv,"astar_endpoint_regression");ros::NodeHandle nh("/scan_planner_node");
 nh.setParam("grid_map/ground_height",-2.5);nh.setParam("grid_map/map_sliding_en",false);nh.setParam("grid_map/require_plan_free",true);
 auto grid=std::make_shared<GridMap>();grid->initMap(nh);prior(*grid);
 AStar search;search.initGridMap(grid,Eigen::Vector3i(100,100,100));Vector3d start(-1.,.1,.4),end(0.,1.2,.4);
 check(grid->getInflateOccupancy(start,0.)==0&&grid->getInflateOccupancy(end,M_PI/2)==0,"elbow endpoint body headings genuinely clear");
 const auto chord=search.AstarSearch(.05,start,end);
 check(chord==INIT_ERR,"chord heading reproduces unavailable narrow endpoint");
 check(search.AstarSearch(.05,start,end,0.,M_PI/2)==SUCCESS,"local endpoint headings recover legal elbow search");
 check(search.getPath().size()>=2,"elbow guidance has a nondegenerate path");
 check(search.AstarSearch(.05,start,start+Vector3d(.015,0.,0.),0.,0.)==SUCCESS&&search.getPath().size()>=2,"sub-voxel endpoints retain checked continuous anchors");
 auto obstacle=std::make_shared<GridMap>();obstacle->initMap(nh);prior(*obstacle,false);obstacle->setOccupied({0.,0.,.4});
 AStar lateral;lateral.initGridMap(obstacle,Eigen::Vector3i(100,100,100));
 // The rectangle prior is intentionally too narrow for the inflated point's
 // side clearance: no optimizer endpoint may be moved beyond the 0.4m limit.
 const auto unavailable=lateral.AstarSearch(.05,{0.,0.,.4},{.9,0.,.4},0.,0.);
 check(unavailable==INIT_ERR||unavailable==SEARCH_ERR,"truly blocked corridor is not opened by endpoint recovery");
 prior(*grid,false);scan_planner::BsplineOptimizer opt;opt.setEnvironment(grid);opt.setParam(nh);opt.a_star_=std::make_shared<AStar>();opt.a_star_->initGridMap(grid,Eigen::Vector3i(100,100,100));
 Eigen::MatrixXd points(3,16);for(int i=0;i<points.cols();++i)points.col(i)=Vector3d(-1.+2.*i/(points.cols()-1),0.,.4);
 opt.initControlPoints(points,true);
 // Introduce a complete wall after initialization to exercise runtime rebound
 // failure, instead of merely checking a function's stored status.
 for(double y=-.7;y<=.7;y+=.05)grid->setOccupied({0.,y,.4});
 const auto before=opt.a_star_->searchCount();const auto begun=ros::SteadyTime::now();
 check(!opt.BsplineOptimizeTrajRebound(points,.4),"new blocking geometry rejects the candidate");
 check(opt.a_star_->searchCount()-before<=3,"failed A-star cannot request twenty empty rebound restarts");
 check((ros::SteadyTime::now()-begun).toSec()<1.,"blocked optimization terminates promptly");
 scan_planner::UniformBspline original(points,3,.4),slower(points,3,.8);
 double error=0.;for(int i=0;i<=100;++i)error=std::max(error,(original.evaluateDeBoorT(original.getTimeSum()*i/100.)-slower.evaluateDeBoorT(slower.getTimeSum()*i/100.)).norm());
 check(error<1e-9,"stationary uniform retiming preserves the optimized geometry");
 auto stationary_grid=std::make_shared<GridMap>();stationary_grid->initMap(nh);prior(*stationary_grid,false);
 scan_planner::BsplineOptimizer stationary;stationary.setParam(nh);stationary.setEnvironment(stationary_grid);
 Eigen::MatrixXd repeated(3,16);for(int i=0;i<repeated.cols();++i)repeated.col(i)=Vector3d(-1.,0.,.4);
 stationary.ref_pts_.assign(14,Vector3d(-1.,0.,.4));Eigen::MatrixXd refined;
 check(stationary.BsplineOptimizeTrajRefine(repeated,.4,refined)&&refined.allFinite(),"repeated stationary fitness references stay finite");
 std::cout<<"No robot interfaces used. Collision dimensions and prior were not relaxed.\n";
}
