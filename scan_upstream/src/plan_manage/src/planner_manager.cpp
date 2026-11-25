#include <plan_manage/trajectory_dynamics.hpp>
// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <algorithm>
#include <cmath>
#include <thread>

namespace scan_upstream_planner
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
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj, UniformBspline &result,
                                        double replan_distance, double prefix_cruise_speed,
                                        const std::function<void(std::vector<Eigen::Vector3d> &)> &shape_seed,
                                        const std::function<bool(UniformBspline &)> &accept_candidate,
                                        const std::function<bool(UniformBspline &)> &accept_retimed)
  {

    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continuous_failures_count_++;
      return false;
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    constexpr size_t max_seed_points = 256;
    constexpr int max_sampling_passes = 12;
    if (!std::isfinite(ts) || ts <= 0.0)
      return false;

    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true;
    const double reuse_time = (t_start - local_data_.start_time_).toSec();
    bool flag_force_polynomial = local_data_.start_time_.isZero() || !std::isfinite(local_data_.duration_) ||
                                 !std::isfinite(reuse_time) || reuse_time < 0.0 ||
                                 reuse_time >= local_data_.duration_ - 1e-3;
    bool flag_regenerate = false;
    do
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
        if (!std::isfinite(time) || time <= 0.0)
          return false;

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

        if (gl_traj.getTimes().empty())
          return false;

        double t;
        bool flag_too_far;
        int sampling_passes = 0;
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
            if (!pt.allFinite() || point_set.size() == max_seed_points)
            {
              ROS_WARN_THROTTLE(1.0, "Rejecting invalid or oversized polynomial trajectory seed.");
              continuous_failures_count_++;
              return false;
            }
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while ((flag_too_far || point_set.size() < 7) && ++sampling_passes < max_sampling_passes);
        if (flag_too_far || point_set.size() < 7)
        {
          ROS_WARN_THROTTLE(1.0, "Polynomial trajectory seed could not be sampled within limits.");
          continuous_failures_count_++;
          return false;
        }
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {
        // Failed or degenerate reuse attempts restart once with a polynomial seed.
        flag_force_polynomial = flag_regenerate = true;
        double t;

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = reuse_time; t < local_data_.duration_ + 1e-3 && segment_point.size() < max_seed_points; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (segment_point.size() > 1)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        if (segment_point.empty() || segment_point.size() == max_seed_points)
          continue;
        t -= ts;

        const double poly_time = (segment_point.back() - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (!std::isfinite(poly_time))
          continue;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(segment_point.back(),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time && segment_point.size() < max_seed_points; t += ts)
          {
            segment_point.push_back(gl_traj.evaluate(t));
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }

        if (segment_point.size() < 2 || segment_point.size() == max_seed_points ||
            !std::isfinite(pseudo_arc_length.back()) || pseudo_arc_length.back() <= 1e-6)
          continue;

        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        int sampling_passes = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          double sample_length = 0.0;
          size_t id = 0;
          while (id + 1 < pseudo_arc_length.size() && sample_length <= pseudo_arc_length.back() &&
                 point_set.size() < max_seed_points)
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          if (point_set.size() == max_seed_points)
            break;
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7 && ++sampling_passes < max_sampling_passes);

        if (point_set.size() < 7 || point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3 ||
            point_set.size() == max_seed_points)
          continue;

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(reuse_time));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(reuse_time));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());
        flag_regenerate = false;
      }
    } while (flag_regenerate);

    applyLinearZReference(point_set, start_pt(2), local_target_pt(2));
    shape_seed(point_set);

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<vector<Eigen::Vector3d>> a_star_paths;
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = ros::Time::now() - t_start;

    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = ros::Time::now() - t_start;
    t_start = ros::Time::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    // Geometry optimization keeps its nominal acceleration cost. Time
    // allocation and acceptance share the actual command/dynamic limits.
    // Otherwise a feasible 2.0 m/s^2 curve gets stretched as if 0.6 were
    // still the final limit, slowing every normally executed prefix.
    const auto scale_needed = [&](UniformBspline &curve) {
      return requiredTimeScale(cubicDynamicBounds(curve), pp_.max_vel_,
          pp_.max_vel_ + pp_.vel_tolerance_, pp_.max_acc_ + pp_.acc_tolerance_);
    };
    bool flag_step_2_success = true;
    double ratio = scale_needed(pos);
    for (int repair = 0; ratio > 1.0 + 1e-6 && repair < 4; ++repair)
    {
      if (!std::isfinite(ratio) || ratio > 3.0)
      {
        flag_step_2_success = false;
        break;
      }
      Eigen::MatrixXd repaired;
      // Retain exact boundary states during a required slowdown. The small
      // margin covers the refit; a fully feasible curve needs no slowdown.
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives,
                                          ratio * 1.015, ts, repaired);
      if (!flag_step_2_success)
        break;
      pos = UniformBspline(repaired, 3, ts);
      ratio = scale_needed(pos);
    }
    if (!flag_step_2_success || !checkDynamicFeasibility(pos))
    {
      ROS_WARN_THROTTLE(1.0, "Refined trajectory remains unsafe or dynamically infeasible after bounded timing repair.");
      continuous_failures_count_++;
      return false;
    }

    t_refine = ros::Time::now() - t_start;

    // A uniform cruise speed-up changes the inherited initial derivatives.
    // Keep the feasible trajectory's boundary states; the prefix retimer
    // below is the single speed-up path and pins the same endpoint jets.
    if (!accept_candidate(pos))
    {
      ++continuous_failures_count_;
      return false;
    }

    if (prefix_cruise_speed > 0.0)
    {
      UniformBspline retimed = pos;
      if (retimeCruisePrefix(retimed, replan_distance, prefix_cruise_speed) &&
          checkDynamicFeasibility(retimed) && accept_retimed(retimed) && accept_candidate(retimed))
      {
        ROS_INFO("Accepted experimental cruise prefix retime.");
        pos = retimed;
      }
    }
    result = pos;

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

    return true;
  }

  void SCANPlannerManager::commitCandidate(const UniformBspline &candidate)
  {
    updateTrajInfo(candidate, ros::Time::now());
    continuous_failures_count_ = 0;
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

    if (gl_traj.getTimes().empty())
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

    if (gl_traj.getTimes().empty())
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

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

  bool SCANPlannerManager::retimeCruisePrefix(UniformBspline &curve, double distance,
                                             double horizontal_target) const
  {
    const double duration = curve.getTimeSum();
    if (!std::isfinite(horizontal_target) || distance <= 0.0 || duration <= 0.1 ||
        pp_.max_acc_ <= 0.0 || pp_.max_vel_ <= 0.0 || duration > 20.0)
      return false;

    struct Sample
    {
      Eigen::Vector3d position;
      double time, arc, speed, original_speed;
    };
    UniformBspline velocity = curve.getDerivative();
    UniformBspline acceleration = velocity.getDerivative();
    const Eigen::Vector3d start = curve.evaluateDeBoorT(0.0);
    const int count = static_cast<int>(std::ceil(duration / 0.04));
    std::vector<Sample> profile;
    profile.reserve(count + 1);
    size_t handoff = 0, end = 0;
    for (int i = 0; i <= count; ++i)
    {
      const double t = duration * i / count;
      const Eigen::Vector3d p = curve.evaluateDeBoorT(t);
      if (!profile.empty() && (p - profile.back().position).norm() < 1e-5)
        continue;
      const double arc = profile.empty() ? 0.0 : profile.back().arc + (p - profile.back().position).norm();
      const Eigen::Vector3d v = velocity.evaluateDeBoorT(t);
      const double speed = v.norm();
      profile.push_back({p, t, arc, speed, speed});
      if (!handoff && (p - start).norm() >= distance)
        handoff = profile.size() - 1;
      if (handoff && arc >= profile[handoff].arc + 0.50)
      {
        end = profile.size() - 1;
        break;
      }
    }
    if (!end)
      return false;

    // Leave room for curvature and the acceleration introduced by fitting.
    const double tangential_limit = 0.55 * pp_.max_acc_;
    for (size_t i = 1; i < end; ++i)
    {
      auto &sample = profile[i];
      const Eigen::Vector3d v = velocity.evaluateDeBoorT(sample.time);
      const double speed = sample.original_speed;
      if (speed < 1e-3 || v.head<2>().norm() < 1e-3)
        continue;
      const Eigen::Vector3d tangent = v / speed;
      if (tangent.dot(sample.position - start) <= 0.0)
        continue;
      const Eigen::Vector3d acc = acceleration.evaluateDeBoorT(sample.time);
      const double curvature = (acc - tangent * tangent.dot(acc)).norm() / (speed * speed);
      const double horizontal_fraction = tangent.head<2>().norm();
      const double cap = std::min(pp_.max_vel_,
          std::sqrt(0.60 * pp_.max_acc_ / std::max(1e-6, curvature)));
      const double ramp = std::clamp(std::min(sample.arc / 0.22,
          (profile[end].arc - sample.arc) / (profile[end].arc - profile[handoff].arc)), 0.0, 1.0);
      const double weight = ramp * ramp * (3.0 - 2.0 * ramp);
      const double requested = std::min(cap, horizontal_target / horizontal_fraction);
      sample.speed = speed + weight * std::max(0.0, requested - speed);
    }
    const auto limited = [&](double v, double ds) {
      return std::sqrt(v * v + 2.0 * tangential_limit * ds);
    };
    for (size_t i = 1; i < end; ++i)
      profile[i].speed = std::min(profile[i].speed,
          limited(profile[i - 1].speed, profile[i].arc - profile[i - 1].arc));
    for (size_t i = end - 1; i > 0; --i)
      profile[i].speed = std::min(profile[i].speed,
          limited(profile[i + 1].speed, profile[i + 1].arc - profile[i].arc));

    std::vector<double> time(end + 1, 0.0);
    for (size_t i = 1; i <= end; ++i)
    {
      const double sum = profile[i - 1].speed + profile[i].speed;
      if (sum <= 1e-4)
        return false;
      time[i] = time[i - 1] + 2.0 * (profile[i].arc - profile[i - 1].arc) / sum;
    }
    const double time_saved = profile[end].time - time.back();
    if (time_saved <= 0.01 || time[handoff] >= profile[handoff].time - 0.01)
      return false;
    const double new_duration = duration - time_saved;
    const int spans = std::max(6, static_cast<int>(std::ceil(new_duration / 0.065)));
    const double dt = new_duration / spans;
    std::vector<Eigen::Vector3d> points;
    points.reserve(spans + 1);
    size_t segment = 1;
    for (int i = 0; i <= spans; ++i)
    {
      const double t = dt * i;
      double original = std::min(duration, t + time_saved);
      if (t < time.back())
      {
        while (time[segment] < t)
          ++segment;
        const double u = (t - time[segment - 1]) / (time[segment] - time[segment - 1]);
        original = (1.0 - u) * profile[segment - 1].time + u * profile[segment].time;
      }
      points.push_back(curve.evaluateDeBoorT(original));
    }

    const std::vector<Eigen::Vector3d> derivatives = {velocity.evaluateDeBoorT(0.0),
        velocity.evaluateDeBoorT(duration), acceleration.evaluateDeBoorT(0.0),
        acceleration.evaluateDeBoorT(duration)};
    Eigen::MatrixXd control;
    UniformBspline::parameterizeToBspline(dt, points, derivatives, control);
    // The fitter uses least squares. Pin the three endpoint controls to
    // reproduce the old position, velocity and acceleration exactly.
    for (int end_index = 0; end_index <= 1; ++end_index)
    {
      const int center = end_index ? control.cols() - 2 : 1;
      const Eigen::Vector3d curvature = derivatives[2 + end_index] * dt * dt;
      const Eigen::Vector3d tangent = derivatives[end_index] * dt;
      control.col(center) = (end_index ? points.back() : points.front()) - curvature / 6.0;
      control.col(center - 1) = control.col(center) - tangent + curvature / 2.0;
      control.col(center + 1) = control.col(center) + tangent + curvature / 2.0;
    }
    UniformBspline candidate(control, 3, dt);
    UniformBspline candidate_v = candidate.getDerivative();
    UniformBspline candidate_a = candidate_v.getDerivative();
    double first_handoff = -1.0;
    const int checks = static_cast<int>(std::ceil(new_duration / 0.01));
    for (int i = 0; i <= checks; ++i)
    {
      const double t = new_duration * i / checks;
      const Eigen::Vector3d v = candidate_v.evaluateDeBoorT(t);
      const Eigen::Vector3d a = candidate_a.evaluateDeBoorT(t);
      double velocity_limit = pp_.max_vel_;
      double acceleration_limit = pp_.max_acc_ + 1e-3;
      // Preserve the original tail, including feasible values that already
      // use the planner's tolerance. The updated prefix uses nominal limits.
      if (t >= time.back())
      {
        const double old_time = t + time_saved;
        velocity_limit = std::max(velocity_limit, velocity.evaluateDeBoorT(old_time).norm());
        acceleration_limit = std::max(acceleration_limit,
            acceleration.evaluateDeBoorT(old_time).norm() + 0.015);
      }
      if (!v.allFinite() || !a.allFinite() || v.head<2>().norm() > pp_.max_vel_ + 1e-3 ||
          v.norm() > velocity_limit + 1e-3 || a.norm() > acceleration_limit)
        return false;
      if (first_handoff < 0.0 && (candidate.evaluateDeBoorT(t) - start).norm() >= distance)
        first_handoff = t;
    }
    // Measure the original crossing at the same time resolution.
    const int original_checks = static_cast<int>(std::ceil(duration / 0.01));
    for (int i = 1; i <= original_checks; ++i)
    {
      const double t = duration * i / original_checks;
      if ((curve.evaluateDeBoorT(t) - start).norm() >= distance)
      {
        if (first_handoff < 0.0 || t - first_handoff <= 0.02)
          return false;
        ROS_INFO("Cruise prefix retime: %.2f m crossing %.3f -> %.3f s", distance, t, first_handoff);
        curve = candidate;
        return true;
      }
    }
    return false;
  }

  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    const auto bounds = cubicDynamicBounds(position_traj);
    const double ratio = requiredTimeScale(bounds, pp_.max_vel_,
        pp_.max_vel_ + pp_.vel_tolerance_, pp_.max_acc_ + pp_.acc_tolerance_);
    if (!std::isfinite(ratio) || ratio > 1.0 + 1e-6)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Dynamic bounds rejected: horizontal=" << bounds.horizontal_velocity
          << "/" << pp_.max_vel_ << ", velocity=" << bounds.velocity << "/"
          << pp_.max_vel_ + pp_.vel_tolerance_ << ", acceleration=" << bounds.acceleration
          << "/" << pp_.max_acc_ + pp_.acc_tolerance_);
      return false;
    }
    return true;
  }

  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    // Uniform scaling preserves the original spatial curve, including short
    // splines with too few knots for lengthenTime's fixed interior indices.
    const Eigen::VectorXd knots = bspline.getKnot();
    bspline.setKnot(knots * ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace scan_upstream_planner
