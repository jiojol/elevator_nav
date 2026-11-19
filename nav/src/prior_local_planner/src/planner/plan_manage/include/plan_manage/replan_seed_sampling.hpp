#pragma once
#include <Eigen/Core>
#include <cmath>
#include <vector>

namespace scan_planner {
// Invalid/expired old curves ask the caller to use its existing polynomial
// initialization. A sampled seed is never an executable trajectory.
inline bool sampleReplanSeed(const std::vector<Eigen::Vector3d>& points,
                             const std::vector<double>& arc,
                             const Eigen::Vector3d& target, double control_distance,
                             std::vector<Eigen::Vector3d>& result) {
  result.clear();
  if (points.size()<2 || arc.size()!=points.size() || !target.allFinite() ||
      !std::isfinite(control_distance) || control_distance<=0. || arc.front()!=0.) return false;
  for (size_t i=0;i<arc.size();++i)
    if (!points[i].allFinite() || !std::isfinite(arc[i]) || (i && arc[i]<arc[i-1])) return false;
  if (arc.back()<=1e-9) return false;
  double spacing=control_distance*1.5;
  for (int attempt=0;attempt<32;++attempt) {
    spacing/=1.5;
    if (!std::isfinite(spacing) || spacing<=1e-12) break;
    result.clear();
    double sample=0.;size_t id=0;
    // size_t subtraction underflows for a one-point old curve. With no arc,
    // repeatedly dividing spacing cannot produce seven valid seed points.
    while (id+1<arc.size() && sample<=arc.back()) {
      if (sample>=arc[id] && sample<arc[id+1]) {
        result.push_back((sample-arc[id])/(arc[id+1]-arc[id])*points[id+1] +
                         (arc[id+1]-sample)/(arc[id+1]-arc[id])*points[id]);
        sample+=spacing;
        if (result.size()>2000) { result.clear();return false; }
      } else ++id;
    }
    result.push_back(target);
    if (result.size()>=7) return true;
  }
  result.clear();return false;
}
}
