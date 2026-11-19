#include <dog_nav_bringup/planar_reference.hpp>
#include <cassert>
#include <iostream>

int main() {
  const Eigen::Vector2d offset(-.221454,-.018929);
  const Eigen::Vector3d centre(.08,9.,-.52);
  // Rotate the sensor about a stationary body through a complete turn. The
  // planning centre and its linear velocity must stay fixed.
  for (int i=0;i<=360;++i) {
    const double yaw=i*std::acos(-1.)/180.;
    const Eigen::Vector2d arm=Eigen::Rotation2Dd(yaw)*offset;
    Eigen::Vector3d sensor=centre;sensor.head<2>()-=arm;
    assert((dog_nav::bodyReference(sensor,yaw,offset)-centre).norm()<1e-12);
    Eigen::Vector3d sensor_velocity(.5*arm.y(),-.5*arm.x(),0.);
    assert(dog_nav::bodyReferenceVelocity(sensor_velocity,.5,yaw,offset).norm()<1e-12);
  }
  // New-goal incident: the old sensor is still facing north while the new
  // path leaves southeast. The route must start at the measured body, not
  // the old sensor translated using the new path heading.
  const Eigen::Vector3d sensor(.013842753,9.225358162,-.511188073);
  const double measured_yaw=1.821680150;
  const auto measured=dog_nav::bodyReference(sensor,measured_yaw,offset);
  std::vector<Eigen::Vector3d> route{{.1,9.3,-.55},{.3,9.1,-.55},{2.3,7.59,-.54}};
  const auto converted=dog_nav::bodyReferenceRoute(route,-3.13,offset,measured);
  assert((converted.front()-measured).norm()<1e-12);
  assert((converted.back()-dog_nav::bodyReference(route.back(),-3.13,offset)).norm()<1e-12);
  for (size_t i=1;i+1<route.size();++i) assert((converted[i]-route[i]).norm()<1e-12);
  const auto false_start=dog_nav::bodyReference(sensor,-.413,offset);
  assert((false_start-measured).head<2>().norm()>.39);
  assert(converted.front().z()==sensor.z());
  const auto single=dog_nav::bodyReferenceRoute({route.back()},-3.13,offset,measured);
  assert((single.back()-converted.back()).norm()<1e-12);
  std::vector<Eigen::Vector3d> line;
  for (int i=0;i<=20;++i) line.emplace_back(.1*i,0.,0.);
  const auto joined=dog_nav::bodyReferenceRoute(line,0.,offset,{-offset.norm(),0.,0.});
  for (size_t i=1;i<joined.size();++i) assert(joined[i].x()>joined[i-1].x());
  std::cout<<"PASS: body centre/velocity invariant during turn; next goal anchored to measured body\n";
}
