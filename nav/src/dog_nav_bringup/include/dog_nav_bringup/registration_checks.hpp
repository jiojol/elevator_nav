#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace dog_nav {
// Compare corrections at the measured robot, not the arbitrary LIO origin.
// Rotating a frame whose origin is far away must not count as robot translation.
inline double registrationShift(const Eigen::Isometry3d& before,
                                const Eigen::Isometry3d& after,
                                const Eigen::Vector3d& local_anchor) {
  return (after*local_anchor-before*local_anchor).norm();
}
inline double registrationRotation(const Eigen::Isometry3d& before,
                                   const Eigen::Isometry3d& after) {
  return Eigen::Quaterniond(before.linear()).angularDistance(Eigen::Quaterniond(after.linear()));
}
inline double registrationTilt(const Eigen::Isometry3d& alignment) {
  return std::acos(std::max(-1.,std::min(1.,alignment(2,2))));
}
}
