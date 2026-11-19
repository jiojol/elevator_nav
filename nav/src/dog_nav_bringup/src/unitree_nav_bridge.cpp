#include <dog_nav_bringup/bridge_control.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <vector>

static double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
class UnitreeNavBridge {
 public:
  UnitreeNavBridge():pnh_("~"),control_(parameter("max_linear_x",.3),parameter("max_linear_y",0.),
                                     parameter("max_angular_z",.5),parameter("command_timeout",.3)) {
    std::string interface;pnh_.param<std::string>("interface",interface,"eth0");
    pnh_.param("auto_enable",auto_enable_,true);
    unitree::robot::ChannelFactory::Instance()->Init(0,interface);
    client_=std::make_unique<unitree::robot::go2::SportClient>();
    client_->SetTimeout(.3f);client_->Init();
    odom_=nh_.advertise<nav_msgs::Odometry>("/go2/odom",10);
    status_=nh_.advertise<std_msgs::String>("/go2_ros_bridge/control_status",1,true);
    for(const auto& topic:{"rt/sportmodestate","rt/lf/sportmodestate"}) {
      auto subscriber=std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(topic);
      subscriber->InitChannel([this](const void* data){feedback(data);},1);
      subscribers_.push_back(subscriber);
    }
    command_=nh_.subscribe("/cmd_vel",1,&UnitreeNavBridge::command,this);
    stop_=nh_.advertiseService("/inspection/stop",&UnitreeNavBridge::stop,this);
    resume_=nh_.advertiseService("/inspection/resume_control",&UnitreeNavBridge::resume,this);
    timer_=nh_.createWallTimer(ros::WallDuration(1./30.),&UnitreeNavBridge::tick,this);
    ROS_INFO("Unitree navigation bridge: interface=%s; real SDK StopMove acknowledgement required",interface.c_str());
  }
  ~UnitreeNavBridge(){
    timer_.stop();command_.shutdown();stop_.shutdown();resume_.shutdown();
    control_.hold();client_->StopMove();
    for(auto& subscriber:subscribers_)subscriber->CloseChannel();
    subscribers_.clear();
  }
 private:
  double parameter(const char* name,double fallback){double value;pnh_.param(name,value,fallback);return value;}
  bool stationary() {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return received_>0.&&steady()-received_<.3&&speed_<.03&&yaw_speed_<.08;
  }
  bool feedbackFresh() {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return received_>0.&&steady()-received_<.3;
  }
  void feedback(const void* data) {
    if(!ros::ok())return;
    const auto& state=*static_cast<const unitree_go::msg::dds_::SportModeState_*>(data);
    const auto& v=state.velocity();const auto& q=state.imu_state().quaternion();
    if(!std::isfinite(v[0])||!std::isfinite(v[1])||!std::isfinite(v[2])||!std::isfinite(state.yaw_speed()))return;
    double norm=0.;
    for(float value:q){if(!std::isfinite(value))return;norm+=value*value;}
    if(std::abs(norm-1.)>.1)return;
    for(float value:state.position())if(!std::isfinite(value))return;
    nav_msgs::Odometry msg;msg.header.stamp=ros::Time::now();msg.header.frame_id="go2_odom";msg.child_frame_id="base_link";
    msg.pose.pose.position.x=state.position()[0];msg.pose.pose.position.y=state.position()[1];msg.pose.pose.position.z=state.position()[2];
    msg.pose.pose.orientation.w=q[0];msg.pose.pose.orientation.x=q[1];msg.pose.pose.orientation.y=q[2];msg.pose.pose.orientation.z=q[3];
    // SDK state.velocity() is already in the body frame. nav_msgs/Odometry
    // twist uses child_frame_id=base_link: preserve vx/vy, do not rotate twice.
    // Command-side vy suppression must never erase real lateral feedback.
    msg.twist.twist.linear.x=v[0];msg.twist.twist.linear.y=v[1];
    msg.twist.twist.linear.z=v[2];msg.twist.twist.angular.z=state.yaw_speed();
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      speed_=std::hypot(v[0],v[1]);yaw_speed_=std::abs(state.yaw_speed());
      mode_=state.mode();body_height_=state.body_height();received_=steady();
    }
    odom_.publish(msg);
  }
  int stopSdk() {
    const int result=client_->StopMove();control_.acknowledge(result);
    last_stop_result_=result;
    // A failed RPC leaves the physical outcome unknown; retry stopping even
    // if no earlier Move was acknowledged.
    moving_=result!=0;
    stop_retry_at_=result==0?0.:steady()+.5;
    if(result!=0){
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      ROS_WARN_THROTTLE(5.,"Unitree StopMove rejected: result=%d, sport mode=%d, body height=%.3f m. Check standing/control mode using remote.",result,mode_,body_height_);
    }
    return result;
  }
  bool stop(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response) {
    starting_=false;control_.hold();
    const int result=stopSdk();response.success=result==0;
    response.message="Unitree SportClient::StopMove result="+std::to_string(result);
    publishStatus();return true;
  }
  bool resume(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response) {
    starting_=false;response.success=control_.arm(stationary());
    response.message=response.success?"Control enabled; waiting for a new command":"Need acknowledged StopMove and fresh stationary hardware feedback";
    publishStatus();return true;
  }
  void command(const geometry_msgs::TwistConstPtr& msg) {
    for(double value:{msg->linear.x,msg->linear.y,msg->linear.z,msg->angular.x,msg->angular.y,msg->angular.z}) {
      if(!std::isfinite(value)){control_.hold();stopSdk();return;}
    }
    if (control_.submit(msg->linear.x, msg->linear.y, msg->angular.z, steady()) &&
        std::abs(msg->linear.x) + std::abs(msg->linear.y) + std::abs(msg->angular.z) > 1.e-9) {
      ++nonzero_commands_;
    }
  }

  void publishStatus() {
    std::ostringstream stream;
    stream<<"{\"control_enabled\":"<<(control_.enabled()?"true":"false")
          <<",\"stop_acknowledged\":"<<(control_.acknowledged()?"true":"false")
          <<",\"hardware_feedback_fresh\":"<<(feedbackFresh()?"true":"false")
          <<",\"state\":\""<<(starting_?"WAITING_CHASSIS":control_.enabled()?"READY":"HELD")<<"\"}";
    auto json=stream.str();json.pop_back();
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      json+=",\"sport_mode\":"+std::to_string(mode_)+",\"body_height\":"+std::to_string(body_height_)
           +",\"last_stop_result\":"+std::to_string(last_stop_result_)
           +",\"last_move_result\":"+std::to_string(last_move_result_)
           +",\"nonzero_commands\":"+std::to_string(nonzero_commands_)
           +",\"moves_acknowledged\":"+std::to_string(moves_acknowledged_)+"}";
    }
    std_msgs::String status;status.data=json;status_.publish(status);
    pnh_.setParam("control_enabled",control_.enabled());
  }
  void tick(const ros::WallTimerEvent&) {
    const double now=steady();
    if(starting_) {
      if(!control_.acknowledged()&&now>=stop_retry_at_)stopSdk();
      if(control_.acknowledged()&&stationary()) {
        starting_=false;
        if(auto_enable_)control_.arm(true);
        ROS_INFO("Unitree bridge ready: StopMove accepted and stationary hardware feedback received");
      } else {
        ROS_WARN_THROTTLE(5.,"WAITING_CHASSIS: StopMove result=%d; waiting for confirmed stop and fresh stationary feedback. Navigation stays online; motion disabled. Stand robot using remote when ready.",last_stop_result_);
      }
      if(starting_){
        if(now-status_at_>.2){publishStatus();status_at_=now;}
        return;
      }
    }
    if (control_.enabled() && !feedbackFresh()) {
      ROS_WARN("Unitree feedback timed out; holding chassis control");
      control_.hold();
      stopSdk();
    }
    const auto cmd=control_.desired(steady());
    if(control_.enabled()&&(std::abs(cmd.x)+std::abs(cmd.y)+std::abs(cmd.yaw)>1.e-9)) {
      // BridgeControl has bounded all three body-frame command axes.
      const int result=client_->Move(cmd.x,cmd.y,cmd.yaw);
      last_move_result_ = result;
      if (result != 0) {
        control_.hold();
        stopSdk();
        ROS_ERROR_THROTTLE(1., "Unitree Move failed: %d", result);
      } else {
        if (!moving_) {
          ROS_INFO("Unitree Move accepted: vx=%.3f vy=%.3f wz=%.3f", cmd.x, cmd.y, cmd.yaw);
        }
        ++moves_acknowledged_;
        moving_ = true;
      }
    } else if(moving_&&now>=stop_retry_at_) {
      const int result=stopSdk();
      if(result!=0)control_.hold();
    }
    if(now-status_at_>.2){publishStatus();status_at_=now;}
  }
  ros::NodeHandle nh_,pnh_;
  dog_nav::BridgeControl control_;
  std::unique_ptr<unitree::robot::go2::SportClient> client_;
  ros::Publisher odom_,status_;ros::Subscriber command_;ros::ServiceServer stop_,resume_;ros::WallTimer timer_;
  std::mutex feedback_mutex_;
  std::vector<unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>> subscribers_;
  double received_=0.,speed_=0.,yaw_speed_=0.,stop_retry_at_=0.,status_at_=0.;
  int mode_=-1,last_stop_result_=-999,last_move_result_=-999;
  unsigned long nonzero_commands_=0,moves_acknowledged_=0;
  double body_height_=0.;
  bool starting_=true,auto_enable_=true,moving_=false;
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"go2_ros_bridge");
  try {UnitreeNavBridge bridge;ros::spin();}
  catch(const std::exception& error){ROS_FATAL("Unitree navigation bridge: %s",error.what());return 1;}
  return 0;
}
