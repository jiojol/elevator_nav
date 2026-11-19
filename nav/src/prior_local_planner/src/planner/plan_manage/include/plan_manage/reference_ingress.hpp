#pragma once
#include <plan_manage/reference_path.hpp>
#include <cmath>

namespace scan_planner {
// Shape only the route initializer. Native rebound optimization and complete
// trajectory validation still decide whether a curve may be published.
template<class Free>
std::vector<Eigen::Vector3d> referenceIngress(const std::vector<Eigen::Vector3d>& points,
                                            double start_yaw,Free free) {
  ReferencePath route;
  if (!std::isfinite(start_yaw) || !route.set(points) || route.length()<.3) return points;
  const Eigen::Vector3d start=points.front(),forward(std::cos(start_yaw),std::sin(start_yaw),0.);
  if (!free(start,start_yaw)) return points;
  std::vector<Eigen::Vector3d> best=points;double best_score=1e9;
  for (double lead:{0.,.05,.10,.15,.20,.35,.50}) for (double join:{.3,.4,.5,.6,.75,.9,1.2,1.6,2.,2.5}) {
    const Eigen::Vector3d p0=start+lead*forward;
    if (join>=route.length()) continue;
    const auto p3=route.at(join);
    auto tangent=route.tangent(join);tangent.z()=0.;
    if (tangent.head<2>().norm()<1e-6) continue;
    tangent.normalize();
    for (double arm:{.05,.10,.15,.25,.4,.6}) {
      const Eigen::Vector3d p1=p0+arm*forward,p2=p3-std::min(.3,join/3.)*tangent;
      std::vector<Eigen::Vector3d> prefix;
      bool safe=true;
      for (int i=0;i<=20;++i) {
        const Eigen::Vector3d p=start+(double(i)/20.)*lead*forward;
        if (!free(p,start_yaw)) { safe=false;break; }
        prefix.push_back(p);
      }
      if (!safe) continue;
      for (int i=1;i<=60;++i) {
        const double u=double(i)/60.,v=1.-u;
        Eigen::Vector3d p=v*v*v*p0+3.*v*v*u*p1+3.*v*u*u*p2+u*u*u*p3;
        // Keep the route's height sequence, including stairs and measured Z.
        p.z()=(1.-u)*start.z()+u*route.at(u*join).z();
        const Eigen::Vector3d d=3.*v*v*(p1-p0)+6.*v*u*(p2-p1)+3.*u*u*(p3-p2);
        if (d.head<2>().norm()<1e-6 || !free(p,std::atan2(d.y(),d.x()))) { safe=false;break; }
        prefix.push_back(p);
      }
      if (!safe) continue;
      const auto tail=route.vertices(join,route.length());
      prefix.insert(prefix.end(),tail.begin()+1,tail.end());
      double score=0.;
      for (size_t i=2;i<prefix.size();++i) {
        const Eigen::Vector3d a=prefix[i-1]-prefix[i-2],b=prefix[i]-prefix[i-1];
        const double ds=.5*(a.head<2>().norm()+b.head<2>().norm());
        if (a.head<2>().norm()<1e-6 || b.head<2>().norm()<1e-6) continue;
        const double turn=std::abs(std::atan2(a.x()*b.y()-a.y()*b.x(),a.head<2>().dot(b.head<2>())));
        score=std::max(score,turn/ds);
      }
      if (score<best_score) { best_score=score;best=prefix; }
    }
  }
  return best;
}
}
