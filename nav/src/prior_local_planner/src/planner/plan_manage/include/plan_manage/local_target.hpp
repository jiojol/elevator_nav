#pragma once
#include <plan_manage/reference_path.hpp>

namespace scan_planner {
// Prefer a continuous terminal neighborhood to an isolated free route point.
// This only chooses a boundary for the native optimizer; it cannot certify a
// route or bypass full spline publication validation.
template<class Free>
double localTargetArc(const ReferencePath& route, const Eigen::Vector3d& start,
                      double progress, double horizon, double span,
                      double resolution, Free free, bool short_terminal=false) {
  if(route.empty()||!start.allFinite()||!std::isfinite(progress)||progress<0.||
     !std::isfinite(horizon)||horizon<=0.||!std::isfinite(span)||span<=0.||
     !std::isfinite(resolution)||resolution<=0.)return -1.;
  const double nominal=std::min(route.length(),progress+horizon);
  const double end=std::min(route.length(),progress+2.*horizon);
  auto usable=[&](double arc) {
    const auto point=route.at(arc), tangent=route.tangent(arc);
    const double minimum=(short_terminal && arc>=route.length()-1e-6)?std::min(.2,.5*resolution):.2;
    return (point-start).norm()>=minimum && free(point,std::atan2(tangent.y(),tangent.x()));
  };
  auto supported=[&](double arc) {
    if(!usable(arc))return false;
    const double low=std::max(progress,arc-span), high=std::min(route.length(),arc+span);
    const int count=std::max(1,int(std::ceil((high-low)/resolution)));
    for(int i=0;i<=count;++i) {
      const double a=low+(high-low)*i/count;
      const auto point=route.at(a), tangent=route.tangent(a);
      if(!free(point,std::atan2(tangent.y(),tangent.x())))return false;
    }
    return true;
  };
  if(supported(nominal))return nominal;
  // Keep the existing 2*horizon bound. Do not repeatedly change the horizon
  // after a failed optimization, and do not move any requested global goal.
  for(double a=nominal+.1;a<=end+1e-6;a+=.1)
    if(supported(std::min(a,end)))return std::min(a,end);
  if(usable(nominal))return nominal;
  for(double a=nominal+.1;a<=end+1e-6;a+=.1)
    if(usable(std::min(a,end)))return std::min(a,end);
  for(double a=nominal-.1;a>progress+.2;a-=.1)
    if(supported(a))return a;
  for(double a=nominal-.1;a>progress+.2;a-=.1)
    if(usable(a))return a;
  return -1.;
}
}
