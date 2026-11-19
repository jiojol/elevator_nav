#include <dog_nav_bringup/registration_checks.hpp>
#include <cassert>

int main() {
  Eigen::Isometry3d before=Eigen::Isometry3d::Identity(),after=before;
  const Eigen::Vector3d anchor(100.,-50.,3.);
  after.linear()=Eigen::AngleAxisd(.08,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  after.translation()=anchor-after.linear()*anchor;
  assert(after.translation().norm()>2.);
  assert(dog_nav::registrationShift(before,after,anchor)<1e-10);
  assert(std::abs(dog_nav::registrationRotation(before,after)-.08)<1e-10);
  after.translation().x()+=.4;
  assert(std::abs(dog_nav::registrationShift(before,after,anchor)-.4)<1e-10);
  after.linear()=Eigen::AngleAxisd(.096,Eigen::Vector3d::UnitY()).toRotationMatrix();
  assert(std::abs(dog_nav::registrationTilt(after)-.096)<1e-10);
}
