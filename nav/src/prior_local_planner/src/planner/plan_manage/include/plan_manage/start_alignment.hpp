#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
namespace scan_planner {
struct StartAlignmentPlan {bool required=false,allowed=false;double target_yaw=0.,signed_angle=0.;};
inline double alignmentWrap(double a){return std::atan2(std::sin(a),std::cos(a));}
inline double alignmentRemaining(double target,double yaw,double selected) {
 double remaining=alignmentWrap(target-yaw);
 if(std::abs(remaining)>.35){if(selected>0.&&remaining<0.)remaining+=2.*M_PI;if(selected<0.&&remaining>0.)remaining-=2.*M_PI;}
 return remaining;
}
template<class Free>
bool alignmentArcFree(const Eigen::Vector3d& position,double yaw,double delta,double padding,Free free) {
  if(!position.allFinite()||!std::isfinite(yaw)||!std::isfinite(delta)||!std::isfinite(padding)||padding<0.)return false;
  const double direction=delta<0.?-1.:1.;const int count=std::max(1,int(std::ceil((std::abs(delta)+2.*padding)/.025)));
  for(int i=0;i<=count;++i)if(!free(position,yaw+direction*(-padding+(std::abs(delta)+2.*padding)*i/count)))return false;
  return true;
}
template<class Free>
StartAlignmentPlan chooseStartAlignment(const Eigen::Vector3d& position,double yaw,const Eigen::Vector3d& ahead,Free free,double enter=2.0,double padding=.2,double minimum_displacement=.2) {
  StartAlignmentPlan plan;const Eigen::Vector2d d=(ahead-position).head<2>();
  if(!position.allFinite()||!ahead.allFinite()||!std::isfinite(yaw)||d.norm()<minimum_displacement)return plan;
  plan.target_yaw=std::atan2(d.y(),d.x());const double short_turn=alignmentWrap(plan.target_yaw-yaw);
  plan.required=std::abs(short_turn)>=enter;if(!plan.required)return plan;
  // Keep the direction fixed across the +/-pi boundary. Try the shorter
  // action first; an alternative is admitted only after its own full check.
  const double alternate=short_turn+(short_turn>=0.?-2.*M_PI:2.*M_PI);
  for(double turn:{short_turn,alternate})if(alignmentArcFree(position,yaw,turn,padding,free)){plan.allowed=true;plan.signed_angle=turn;break;}
  return plan;
}
}
