#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>

// Nominal whole-robot horizontal envelope, including a configurable margin.
// SAT tests every grid cell intersected by the oriented rectangle, so neither
// a circumscribed disk nor a sparse rim sampling sets the doorway width.
struct RobotFootprint {
  double length=.70,width=.35,margin=.03,center_x=0.,center_y=0.;
  void validate() const {
    for (double v:{length,width,margin,center_x,center_y})
      if (!std::isfinite(v)) throw std::invalid_argument("non-finite Go2 geometry");
    if (length<=0. || width<=0. || margin<0.) throw std::invalid_argument("invalid Go2 geometry");
  }
  double radius() const { return std::hypot(.5*length+margin,.5*width+margin); }
  template<class Visit>
  bool cells(const Eigen::Vector3d& body,double yaw,double resolution,Visit visit) const {
    if (!body.allFinite() || !std::isfinite(yaw) || !std::isfinite(resolution) || resolution<=0.) return false;
    const double c=std::cos(yaw),s=std::sin(yaw),ac=std::abs(c),as=std::abs(s);
    const double hx=.5*length+margin,hy=.5*width+margin,h=.5*resolution;
    const double cx=body.x()+c*center_x-s*center_y,cy=body.y()+s*center_x+c*center_y;
    const double ex=ac*hx+as*hy,ey=as*hx+ac*hy;
    for (int ix=static_cast<int>(std::floor((cx-ex)/resolution));ix<=static_cast<int>(std::floor((cx+ex)/resolution));++ix)
      for (int iy=static_cast<int>(std::floor((cy-ey)/resolution));iy<=static_cast<int>(std::floor((cy+ey)/resolution));++iy) {
        const double dx=(ix+.5)*resolution-cx,dy=(iy+.5)*resolution-cy;
        if (std::abs(dx)>ex+h || std::abs(dy)>ey+h
            || std::abs(c*dx+s*dy)>hx+h*(ac+as)
            || std::abs(-s*dx+c*dy)>hy+h*(ac+as)) continue;
        if (!visit(ix,iy)) return false;
      }
    return true;
  }
};
