#pragma once
#include <plan_manage/reference_path.hpp>
#include <Eigen/Geometry>

namespace scan_planner {
// The saved route and goal are expressed at the navigation reference (lidar
// in legacy mode). A final yaw rotates that point about the body center.
// Keep the requested reference pose, and adapt only the terminal approach:
// P_approach = P_goal + R_goal*offset - R_approach*offset.
// Here offset is navigation-reference -> body, expressed in heading axes.
inline std::vector<Eigen::Vector3d> terminalReference(
    const std::vector<Eigen::Vector3d>& points,double goal_yaw,
    const Eigen::Vector2d& offset) {
  ReferencePath route;
  if (!route.set(points)||!std::isfinite(goal_yaw)||!offset.allFinite()) return {};
  if (offset.norm()<1e-9) return points;
  Eigen::Vector3d tangent=Eigen::Vector3d::Zero();
  for (size_t i=points.size()-1;i>0;--i) {
    tangent=points[i]-points[i-1];
    if (tangent.head<2>().norm()>1e-6) break;
  }
  if (tangent.head<2>().norm()<=1e-6) return points;
  const double approach_yaw=std::atan2(tangent.y(),tangent.x());
  const Eigen::Vector2d delta=Eigen::Rotation2Dd(goal_yaw)*offset-Eigen::Rotation2Dd(approach_yaw)*offset;
  if (delta.norm()<1e-6) return points;
  const double blend=std::min(route.length(),std::max(1.,2.*delta.norm()));
  const double start=route.length()-blend;
  std::vector<Eigen::Vector3d> result;
  if (start>1e-6) result=route.vertices(0.,start);
  const int samples=std::max(2,int(std::ceil(blend/.05)));
  for (int i=0;i<=samples;++i) {
    const double u=double(i)/samples;
    Eigen::Vector3d p=route.at(start+blend*u);
    p.head<2>()+=(u*u*(3.-2.*u))*delta;
    if (result.empty()||(p-result.back()).norm()>1e-6) result.push_back(p);
  }
  return result;
}
}
