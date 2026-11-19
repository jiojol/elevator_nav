#include <plan_manage/replan_seed_sampling.hpp>
#include <cassert>
#include <chrono>
#include <limits>

// Compatibility oracle: the previous sampler, used ONLY on nondegenerate curves.
static std::vector<Eigen::Vector3d> legacy(const std::vector<Eigen::Vector3d>& p,
    const std::vector<double>& arc, const Eigen::Vector3d& goal, double control) {
  double spacing=control*1.5;std::vector<Eigen::Vector3d> out;
  do {
    spacing/=1.5;out.clear();double sample=0;size_t id=0;
    while (id<=arc.size()-2 && sample<=arc.back()) {
      if (sample>=arc[id] && sample<arc[id+1]) {
        out.push_back((sample-arc[id])/(arc[id+1]-arc[id])*p[id+1] +
                      (arc[id+1]-sample)/(arc[id+1]-arc[id])*p[id]);
        sample+=spacing;
      } else ++id;
    }
    out.push_back(goal);
  } while(out.size()<7);
  return out;
}

int main() {
  using Eigen::Vector3d;
  using scan_planner::sampleReplanSeed;
  std::vector<Vector3d> result;
  const Vector3d goal(-1.25,-.15,.4);
  const auto start=std::chrono::steady_clock::now();
  // Exact frozen failure: one remaining old-curve point, zero accumulated arc.
  assert(!sampleReplanSeed({Vector3d(-1.2416776068,-.149930409,.4)}, {0.}, goal,.2,result));
  assert(result.empty());
  assert(!sampleReplanSeed({}, {}, goal,.2,result));
  assert(!sampleReplanSeed({goal,goal}, {0.,0.}, goal,.2,result));
  assert(!sampleReplanSeed({goal,goal}, {0.}, goal,.2,result));
  assert(!sampleReplanSeed({goal,goal}, {0.,-1.}, goal,.2,result));
  assert(!sampleReplanSeed({goal,goal}, {0.,std::numeric_limits<double>::infinity()},goal,.2,result));
  assert(!sampleReplanSeed({goal,goal}, {0.,1.},goal,0.,result));
  assert(std::chrono::steady_clock::now()-start<std::chrono::seconds(1));
  // Ordinary seeds keep the original spacing/interpolation and exact endpoint.
  assert(sampleReplanSeed({Vector3d(0,0,0),Vector3d(1,0,0)}, {0.,1.},Vector3d(1,0,0),.1,result));
  assert(result.size()>=7);
  assert(result.front().norm()<1e-12);
  assert((result[1]-Vector3d(.1,0,0)).norm()<1e-12);
  assert((result.back()-Vector3d(1,0,0)).norm()<1e-12);
  // Repeated vertices are skipped without division by zero.
  assert(sampleReplanSeed({Vector3d::Zero(),Vector3d::Zero(),Vector3d(1,0,0)}, {0.,0.,1.},Vector3d(1,0,0),.1,result));
  for (const auto& p:result) assert(p.allFinite());
  for (double length : {.004,.1,1.,3.5}) {
    const std::vector<Vector3d> p={Vector3d::Zero(),Vector3d(length,0,0)};
    const std::vector<double> arc={0.,length};
    const auto old=legacy(p,arc,p.back(),.2);
    assert(sampleReplanSeed(p,arc,p.back(),.2,result));
    assert(result.size()==old.size());
    for(size_t i=0;i<old.size();++i)assert((old[i]-result[i]).norm()<1e-12);
  }
}
