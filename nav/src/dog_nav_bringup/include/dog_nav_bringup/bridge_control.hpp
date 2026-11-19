#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dog_nav {
struct BridgeVelocity {double x=0.,y=0.,yaw=0.;};
class BridgeControl {
 public:
  BridgeControl(double linear,double lateral,double yaw,double timeout)
      : linear_(linear),lateral_(lateral),yaw_(yaw),timeout_(timeout) {
    if(!std::isfinite(linear)||linear<=0.||!std::isfinite(lateral)||lateral<0.
       ||!std::isfinite(yaw)||yaw<=0.||!std::isfinite(timeout)||timeout<=0.)
      throw std::invalid_argument("invalid bridge command limits");
  }
  void hold(){enabled_=false;ack_=false;have_=false;command_={};}
  void acknowledge(int result){ack_=result==0;}
  bool arm(bool stationary_and_fresh) {
    if(!ack_||!stationary_and_fresh)return false;
    enabled_=true;have_=false;command_={};return true;
  }
  bool submit(double x,double y,double yaw,double now) {
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||!std::isfinite(now)) {
      hold();return false;
    }
    if(!enabled_)return false;
    // Forward every permitted body axis; the selected profile owns its limits.
    command_={clamp(x,linear_),clamp(y,lateral_),clamp(yaw,yaw_)};
    received_=now;have_=true;return true;
  }
  BridgeVelocity desired(double now) const {
    if(!enabled_||!have_||now<received_||now-received_>timeout_)return {};
    return command_;
  }
  bool enabled()const{return enabled_;}
  bool acknowledged()const{return ack_;}
 private:
  static double clamp(double value,double limit){return std::max(-limit,std::min(limit,value));}
  double linear_,lateral_,yaw_,timeout_,received_=0.;
  bool enabled_=false,ack_=false,have_=false;
  BridgeVelocity command_;
};
}
