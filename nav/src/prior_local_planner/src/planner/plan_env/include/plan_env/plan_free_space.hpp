#pragma once
#include <Eigen/Core>
#include <plan_env/robot_footprint.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>

// A static traversability envelope, separate from SCAN's live obstacle map.
// Heights are ground heights. Queries use the shared Go2 footprint.
// This mask never clears live occupancy.
class PlanFreeSpace {
 public:
  void clear() { layers_.clear(); }
  bool ready() const { return !layers_.empty(); }
  void set(const std::vector<Eigen::Vector3d>& points, double resolution,
           const RobotFootprint& footprint, double height_tolerance, double terrain_max_step=0.) {
    clear();resolution_=resolution;height_tolerance_=height_tolerance;footprint_=footprint;
    terrain_max_step_=terrain_max_step;
    if (!std::isfinite(terrain_max_step_) || terrain_max_step_<0.) throw std::invalid_argument("invalid terrain step");
    footprint_.validate();
    if (!std::isfinite(resolution) || resolution<=0. || !std::isfinite(height_tolerance) || height_tolerance<=0.) return;
    for (const auto& p:points) if (p.allFinite()) layers_[key(p.x(),p.y())].push_back(p.z());
  }
  bool contains(const Eigen::Vector3d& body,double yaw,double body_height) const {
    if (!ready()) return false;
    const double ground=body.z()-body_height;
    if (terrainEnabled()) return containsTerrain(body,yaw,ground);
    return footprint_.cells(body,yaw,resolution_,[&](int ix,int iy) {
      const auto found=layers_.find(keyIndex(ix,iy));
      if (found==layers_.end()) return false;
      for (double height:found->second) if (heightMatches(height,ground)) return true;
      return false;
    });
  }
  bool terrainEnabled() const { return terrain_max_step_>0.; }
  bool terrainHeight(double x,double y,double hint,double& height) const {
    const auto it=layers_.find(key(x,y));
    if (!terrainEnabled() || it==layers_.end() || it->second.size()!=1) return false;
    height=it->second.front();
    return heightMatches(height,hint);
  }
 private:
  // Only the explicit single-surface kinematic profile uses this rule. The
  // body remains near its centre support, while each footprint cell may lie
  // on another step of the same locally connected surface. Missing cells and
  // risers above the configured limit still reject the whole footprint.
  bool containsTerrain(const Eigen::Vector3d& body,double yaw,double ground) const {
    double centre;
    if (!terrainHeight(body.x(),body.y(),ground,centre)) return false;
    std::unordered_map<uint64_t,std::pair<int32_t,int32_t>> cells;
    if (!footprint_.cells(body,yaw,resolution_,[&](int ix,int iy) {
      const auto k=keyIndex(ix,iy);const auto it=layers_.find(k);
      if (it==layers_.end() || it->second.size()!=1) return false;
      cells.emplace(k,std::make_pair(ix,iy));return true;
    })) return false;
    const auto first=key(body.x(),body.y());
    if (!cells.count(first)) return false;
    std::queue<uint64_t> pending;std::unordered_set<uint64_t> seen;
    pending.push(first);seen.insert(first);
    const int dx[]={-1,1,0,0},dy[]={0,0,-1,1};
    while (!pending.empty()) {
      const auto k=pending.front();pending.pop();const auto xy=cells.at(k);
      const double z=layers_.at(k).front();
      for(int i=0;i<4;++i) {
        const auto next=keyIndex(xy.first+dx[i],xy.second+dy[i]);
        if (!cells.count(next) || seen.count(next)) continue;
        if (std::abs(layers_.at(next).front()-z)>terrain_max_step_+1e-6) continue;
        seen.insert(next);pending.push(next);
      }
    }
    return seen.size()==cells.size();
  }
  bool heightMatches(double center,double ground) const {
    // FREE PCD entries represent voxel volumes (generator: (key+.5)*res).
    // Apply the configured tolerance to the voxel interval, not its center.
    const double distance=std::max(0.,std::abs(center-ground)-.5*resolution_);
    return distance<=height_tolerance_;
  }
  uint64_t key(double x,double y) const {
    const auto ix=static_cast<int32_t>(std::floor(x/resolution_));
    const auto iy=static_cast<int32_t>(std::floor(y/resolution_));
    return keyIndex(ix,iy);
  }
  static uint64_t keyIndex(int32_t ix,int32_t iy) { return (uint64_t(static_cast<uint32_t>(ix))<<32)|static_cast<uint32_t>(iy); }
  RobotFootprint footprint_;
  double resolution_=0.2,height_tolerance_=0.3,terrain_max_step_=0.;
  std::unordered_map<uint64_t,std::vector<double>> layers_;
};
