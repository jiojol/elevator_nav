#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <vector>
#include <cmath>
#include <limits>

namespace dog_nav {
// Preserve the saved map's vertical reference. Only XY changes from the
// navigation sensor to the body centre; this is not a new 3D calibration.
inline Eigen::Vector3d bodyReference(Eigen::Vector3d point,double yaw,const Eigen::Vector2d& offset) {
  point.head<2>()+=Eigen::Rotation2Dd(yaw)*offset;
  return point;
}
inline Eigen::Vector3d bodyReferenceVelocity(Eigen::Vector3d world_velocity,double world_yaw_rate,
                                           double yaw,const Eigen::Vector2d& offset) {
  const Eigen::Vector2d arm=Eigen::Rotation2Dd(yaw)*offset;
  world_velocity.head<2>()+=world_yaw_rate*Eigen::Vector2d(-arm.y(),arm.x());
  return world_velocity;
}
inline std::vector<Eigen::Vector3d> bodyReferenceRoute(const std::vector<Eigen::Vector3d>& points,
    double goal_yaw,const Eigen::Vector2d& offset,const Eigen::Vector3d& measured_start) {
  auto result=points;
  // Interior FREE-graph vertices are positions, not sensor poses. Inferring
  // a yaw from each 8-neighbour grid edge and rotating the lever arm creates
  // artificial zigzags. Preserve the seed and join its final metre smoothly
  // to the requested body goal, rather than moving only the last vertex and
  // creating a short backwards-facing tail.
  if (!result.empty()) {
    std::vector<double> arc(points.size(),0.);
    for (size_t i=1;i<points.size();++i) arc[i]=arc[i-1]+(points[i]-points[i-1]).norm();
    const double blend=std::min(arc.back(),std::max(1.,2.*offset.norm()));
    const Eigen::Vector2d delta=Eigen::Rotation2Dd(goal_yaw)*offset;
    for (size_t i=0;i<points.size();++i) {
      const double u=blend>1e-9?std::max(0.,std::min(1.,(arc[i]-arc.back()+blend)/blend)):1.;
      result[i].head<2>()+=(u*u*(3.-2.*u))*delta;
    }
  }
  // A same-goal recovery can resend the entire old route. Remove its walked
  // prefix BEFORE anchoring it to the measured body; replacing only the first
  // vertex creates a long backwards leg to the old second vertex.
  if (result.size()>1) {
    double best=std::numeric_limits<double>::infinity();size_t begin=0;
    for(size_t i=1;i<result.size();++i) {
      Eigen::Vector3d d=result[i]-result[i-1],e=measured_start-result[i-1];
      d.z()*=2.;e.z()*=2.; // Preserve the measured story at overlapping XY.
      const double t=d.squaredNorm()>1e-12?std::clamp(e.dot(d)/d.squaredNorm(),0.,1.):0.;
      const double error=(e-t*d).squaredNorm();
      if(error<best) { best=error;begin=i-1; }
    }
    result.erase(result.begin(),result.begin()+begin);
    result.front()=measured_start;
  }
  return result;
}
}
