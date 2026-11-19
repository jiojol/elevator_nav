// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include <plan_manage/reference_timing.hpp>
#include <plan_manage/reference_ingress.hpp>
#include <plan_manage/replan_seed_sampling.hpp>

namespace scan_planner
{
  namespace
  {
    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      if (points.empty())
        return;

      if (points.size() == 1)
      {
        points.front()(2) = start_z;
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        points[i](2) = start_z + ratio * (target_z - start_z);
      }

      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }
  } // namespace

  // SECTION interfaces for setup and query

  SCANPlannerManager::SCANPlannerManager() {}

  SCANPlannerManager::~SCANPlannerManager() { std::cout << "des manager" << std::endl; }

  void SCANPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("optimization/vel_tolerance", pp_.vel_tolerance_, 1.0);
    nh.param("optimization/acc_tolerance", pp_.acc_tolerance_, 1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizon_, 5.0);

    local_data_.traj_id_ = 0;
    local_data_.duration_ = 0.;
    local_data_.start_pos_.setZero();
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(nh);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    // Continuous goals can need a sub-voxel final approach on any surface.
    // The execution adapter, not a voxel edge, declares measured arrival.
    const double minimum_plan_distance=std::min(.2,.05*grid_map_->getResolution());
    if ((start_pt - local_target_pt).norm() < minimum_plan_distance)
    {
      cout << "Close to goal" << endl;
      continuous_failures_count_++;
      return false;
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    const bool route_seed=local_reference_.size()>=2 && !flag_randomPolyTraj && (flag_polyInit || local_data_.duration_<=0.);
    if (route_seed) {
      flag_first_call=false;flag_force_polynomial=false;
      // The prior route supplies only initialization. Native rebound/A* and
      // LBFGS decide the detour; no separate detour planner runs beforehand.
      double max_yaw=.8;ros::param::param("/closed_loop_controller/max_vyaw",max_yaw,.8);
      const bool stationary=start_vel.norm()<.03 && std::isfinite(measured_start_yaw_);
      const auto reference=stationary?referenceIngress(local_reference_,measured_start_yaw_,
          [&](const Eigen::Vector3d& p,double h) { return grid_map_->getInflateOccupancy(p,h)==0; }):local_reference_;
      // At a near-stationary start, millimetre LIO jitter must not set
      // the first spline tangent to a random direction across the prior edge.
      if (stationary) start_vel.setZero();
      const auto seed=timeReference(reference,pp_.max_vel_,pp_.max_acc_,
                                    start_vel.norm(),local_target_vel.norm(),max_yaw,.30);
      if (seed.points.size()<7 || seed.points.size()>1998) return false;
      point_set=seed.points;ts=seed.interval;
      Eigen::Vector3d end_velocity=local_target_vel;
      if (end_velocity.norm()>seed.end_speed && end_velocity.norm()>1e-9) end_velocity*=seed.end_speed/end_velocity.norm();
      start_end_derivatives={start_vel,end_velocity,start_acc,Eigen::Vector3d::Zero()};
    } else do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        if (segment_point.empty()) {
          flag_force_polynomial = flag_regenerate = true;
          continue;
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continuous_failures_count_++;
              return false;
            }
          }
        }

        if (!sampleReplanSeed(segment_point, pseudo_arc_length, local_target_pt,
                              pp_.ctrl_pt_dist, point_set)) {
          flag_force_polynomial = flag_regenerate = true;
          continue;
        }

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // A saved 3D route retains its stair heights; native polynomial mode
    // continues to use SCAN's original linear-height search convention.
    if (!route_seed) applyLinearZReference(point_set, start_pt(2), local_target_pt(2));

    UniformBspline pos;
    bool accepted=false;
    for(int seed_attempt=0;seed_attempt<2;++seed_attempt) {
      if(seed_attempt==1) {
        if(!route_seed)break;
        // A feasible native A-star guide may still be needed when the global
        // reference crosses a new obstacle. It supplies initialization only;
        // no grid path or unoptimized spline can be published.
        const double begin_yaw=std::isfinite(measured_start_yaw_)?measured_start_yaw_:std::atan2(start_vel.y(),start_vel.x());
        const double end_yaw=local_target_vel.head<2>().norm()>1e-6?std::atan2(local_target_vel.y(),local_target_vel.x()):
          std::atan2((local_target_pt-start_pt).y(),(local_target_pt-start_pt).x());
        auto& search=bspline_optimizer_rebound_->a_star_;
        if(search->AstarSearch(grid_map_->getResolution(),start_pt,local_target_pt,begin_yaw,end_yaw)!=ASTAR_RET::SUCCESS)break;
        auto guide=search->getPath();if(guide.size()<2)break;
        guide.front()=start_pt;guide.back()=local_target_pt;
        double max_yaw=.8;ros::param::param("/closed_loop_controller/max_vyaw",max_yaw,.8);
        if(start_vel.norm()<.03)guide=referenceIngress(guide,begin_yaw,[&](const Eigen::Vector3d& p,double h){return grid_map_->getInflateOccupancy(p,h)==0;});
        const auto seed=timeReference(guide,pp_.max_vel_,pp_.max_acc_,start_vel.norm(),local_target_vel.norm(),max_yaw,std::min(.15,grid_map_->getResolution()/std::max(.1,pp_.max_vel_)));
        if(seed.points.size()<7||seed.points.size()>1998)break;
        point_set=seed.points;ts=seed.interval;
        Eigen::Vector3d end_velocity=local_target_vel;
        // A detour must not inherit the old global route's incompatible
        // terminal tangent. Keep its speed cap, use the checked guide approach.
        size_t approach=guide.size()-2;
        while(approach>0&&(guide.back()-guide[approach]).head<2>().norm()<.2)--approach;
        const Eigen::Vector2d incoming=(guide.back()-guide[approach]).head<2>();
        if(incoming.norm()>1e-6&&grid_map_->getInflateOccupancy(local_target_pt,std::atan2(incoming.y(),incoming.x()))==0)
          end_velocity.head<2>()=incoming.normalized()*local_target_vel.head<2>().norm();
        else end_velocity.setZero();
        if(end_velocity.norm()>seed.end_speed&&end_velocity.norm()>1e-9)end_velocity*=seed.end_speed/end_velocity.norm();
        start_end_derivatives={start_vel,end_velocity,start_acc,Eigen::Vector3d::Zero()};
        ROS_INFO_THROTTLE(1.,"Retry native optimization from its full A-star guide; full publication checks remain required.");
      }
      // A forward-only terrain tracker cannot follow tiny reversals introduced
      // solely to smooth the initialization. Keep the native fitting term
      // active for its supplied route, as already done for A-star retries.
      bspline_optimizer_rebound_->setSeedFitness(seed_attempt==1 || (route_seed && grid_map_->terrainEnabled()));
      Eigen::MatrixXd ctrl_pts;
      // Reused old-curve samples are only a seed. Never inherit their shifted
      // origin or old velocity as the boundary of the newly executed curve.
      point_set.front()=start_pt;point_set.back()=local_target_pt;
      start_end_derivatives[0]=start_vel;start_end_derivatives[2]=start_acc;
      UniformBspline::parameterizeToBspline(ts,point_set,start_end_derivatives,ctrl_pts);
      // With zero measured velocity/acceleration the first three controls
      // coincide. The fourth determines the initial direction of motion;
      // leaving it free lets optimization turn the body instantaneously.
      const bool lock_heading=grid_map_->terrainEnabled() && start_vel.norm()<1e-6 &&
          start_acc.norm()<1e-6 && std::isfinite(measured_start_yaw_) && ctrl_pts.cols()>7;
      bspline_optimizer_rebound_->fixStationaryHeading(lock_heading);
      if(lock_heading) {
        const double lead=std::max(1e-3,(ctrl_pts.col(3)-start_pt).head<2>().norm());
        ctrl_pts(0,3)=start_pt.x()+lead*std::cos(measured_start_yaw_);
        ctrl_pts(1,3)=start_pt.y()+lead*std::sin(measured_start_yaw_);
      }
      const auto a_star_paths=bspline_optimizer_rebound_->initControlPoints(ctrl_pts,true);
      t_init=ros::Time::now()-t_start;
      static int vis_id=0;
      visualization_->displayInitPathList(point_set,.2,0);
      visualization_->displayAStarList(a_star_paths,vis_id);
      t_start=ros::Time::now();
      if(!bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts,ts))continue;
      t_opt=ros::Time::now()-t_start;t_start=ros::Time::now();
      pos=UniformBspline(ctrl_pts,3,ts);
      pos.setPhysicalLimits(pp_.max_vel_,pp_.max_acc_,pp_.feasibility_tolerance_);
      double ratio;bool refined=true;
      if(!pos.checkFeasibility(ratio,false)) {
        if(start_vel.norm()<1e-6&&start_acc.norm()<1e-6&&std::isfinite(ratio)&&ratio>1.) {
          // At a confirmed stationary start, uniform knot scaling preserves
          // the optimized geometry exactly and keeps zero boundary motion.
          // Partial-knot stretching followed by a fit can cut across FREE.
          ts*=ratio*1.01;pos=UniformBspline(pos.getControlPoint(),3,ts);
          pos.setPhysicalLimits(pp_.max_vel_,pp_.max_acc_,pp_.feasibility_tolerance_);
          double remaining_ratio;refined=pos.checkFeasibility(remaining_ratio,false);
        } else {
          Eigen::MatrixXd optimal;
          refined=refineTrajAlgo(pos,start_end_derivatives,ratio,ts,optimal);
          if(refined)pos=UniformBspline(optimal,3,ts);
        }
      }
      if(!refined||!checkDynamicFeasibility(pos)||!checkSpatialFeasibility(pos))continue;
      t_refine=ros::Time::now()-t_start;accepted=true;break;
    }
    if(!accepted){continuous_failures_count_++;return false;}

    // save planned results
    updateTrajInfo(pos, ros::Time::now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    return true;
  }

  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    if (waypoints.empty())
      return false;

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    UniformBspline safe_reference=traj;
    const double safe_duration=safe_reference.getTimeSum();
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(safe_reference.evaluateDeBoorT(std::min(safe_duration,t/traj.getTimeSum()*safe_duration)));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      if (vel.norm() > vel_limit)
      {
        ROS_WARN_STREAM("Dynamic feasibility check failed: velocity limit exceeded at t="
                        << tc << ", |v|=" << vel.norm() << " > " << vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      if (acc.norm() > acc_limit)
      {
        ROS_WARN_STREAM("Dynamic feasibility check failed: acceleration limit exceeded at t="
                        << tc << ", |a|=" << acc.norm() << " > " << acc_limit);
        return false;
      }
    }

    return true;
  }

  bool SCANPlannerManager::checkSpatialFeasibility(const UniformBspline& trajectory)
  {
    if (!grid_map_->hasPlanFree()) return false;
    // Native rebound's rolling 2/3 checks are for optimization/replanning.
    // The entire candidate that will actually be published, including its
    // terminal third and endpoint, must satisfy the same occupancy/prior query.
    UniformBspline curve=trajectory;
    auto velocity=curve.getDerivative();
    const auto controls=velocity.getControlPoint();
    double speed=0.;for (int i=0;i<controls.cols();++i) speed=std::max(speed,controls.col(i).norm());
    const double duration=curve.getTimeSum();
    if (!std::isfinite(duration)||duration<=0.||!std::isfinite(speed)) return false;
    const int count=std::max(1,int(std::ceil(duration/std::min(.05,.5*grid_map_->getResolution()/std::max(.01,speed)))));
    if (count>200000) return false;
    for (int i=0;i<=count;++i) {
      const double t=duration*i/count;
      const auto point=curve.evaluateDeBoorT(t);
      Eigen::Vector3d heading=velocity.evaluateDeBoorT(t);
      if (heading.head<2>().norm()<1e-6) heading=curve.evaluateDeBoorT(std::min(duration,t+.02))-curve.evaluateDeBoorT(std::max(0.,t-.02));
      const double yaw=std::atan2(heading.y(),heading.x());
      if (!point.allFinite() || grid_map_->getInflateOccupancy(point,yaw)!=0) {
        ROS_WARN_THROTTLE(1.,"SCAN candidate rejected at t=%.3f/%.3f (%.1f%%), p=(%.3f,%.3f,%.3f), yaw=%.3f: prior=%d obstacle=%d; candidate not published.",t,duration,100.*t/duration,point.x(),point.y(),point.z(),yaw,int(grid_map_->isPlanFree(point,yaw)),grid_map_->getObstacleOccupancy(point,yaw));return false;
      }
    }
    return true;
  }

  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    const Eigen::Vector3d boundary_start=bspline.evaluateDeBoorT(0.);
    const Eigen::Vector3d boundary_end=bspline.evaluateDeBoorT(time_origin);
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    point_set.front()=boundary_start;point_set.back()=boundary_end;
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace scan_planner
