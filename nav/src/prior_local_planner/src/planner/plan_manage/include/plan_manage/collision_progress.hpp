#pragma once
#include <plan_manage/reference_path.hpp>
#include <bspline_opt/uniform_bspline.h>

namespace scan_planner {
// Locate the measured body on the active curve. A held execution clock is
// not a substitute for physical progress when deciding where braking begins.
class CollisionProgress {
 public:
  void reset(UniformBspline curve,double resolution,bool planar=false) {
    planar_=planar;
    path_.clear();arcs_.clear();times_.clear();progress_=0.;have_pose_=false;
    auto velocity=curve.getDerivative();double bound=0.;
    const auto controls=velocity.getControlPoint();
    for(int i=0;i<controls.cols();++i)bound=std::max(bound,controls.col(i).norm());
    const double duration=curve.getTimeSum();
    if(!std::isfinite(duration)||duration<=0.||!std::isfinite(bound)||resolution<=0.)return;
    const int count=std::max(1,int(std::ceil(duration/std::min(.05,.5*resolution/std::max(.01,bound)))));
    if(count>200000)return;
    std::vector<Eigen::Vector3d> points;
    for(int i=0;i<=count;++i) {
      const double t=duration*i/count;const auto point=curve.evaluateDeBoorT(t);
      if(points.empty()||(point-points.back()).norm()>1e-6) {
        arcs_.push_back(points.empty()?0.:arcs_.back()+(point-points.back()).norm());
        points.push_back(point);times_.push_back(t);
      }
    }
    if(!path_.set(points)){arcs_.clear();times_.clear();}
  }
  bool ready() const {return !path_.empty();}
  double projectTime(const Eigen::Vector3d& position) {
    if(!ready())return 0.;
    const double traveled=have_pose_?(position-previous_).norm():.5;
    const auto projection=path_.project(position,std::max(0.,progress_-.1),
        std::min(path_.length(),progress_+traveled+.35),planar_);
    progress_=std::max(progress_,projection.arc);previous_=position;have_pose_=true;
    auto upper=std::upper_bound(arcs_.begin(),arcs_.end(),progress_);
    if(upper==arcs_.end())return times_.back();
    const auto i=std::max<size_t>(1,upper-arcs_.begin());
    return times_[i-1]+(times_[i]-times_[i-1])*(progress_-arcs_[i-1])/(arcs_[i]-arcs_[i-1]);
  }
 private:
  ReferencePath path_;
  std::vector<double> arcs_,times_;
  double progress_=0.;bool have_pose_=false,planar_=false;
  Eigen::Vector3d previous_=Eigen::Vector3d::Zero();
};
inline double collisionStopDistance(double speed,double acceleration,double map_age,
                                    double resolution,double emergency_time) {
  // Keep the existing emergency horizon as reaction/search reserve, then add
  // the distance the measured chassis needs to brake. The horizon alone is
  // not a braking model for a moving Go2.
  const double reaction=std::max(emergency_time,std::max(0.,map_age)+.10);
  return speed*speed/(2.*acceleration)+speed*reaction+resolution;
}
}
