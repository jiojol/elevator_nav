#ifndef DOG_NAV_RVIZ_GOAL3D_TOOL_H
#define DOG_NAV_RVIZ_GOAL3D_TOOL_H

#ifndef Q_MOC_RUN
#include <QObject>
#include <ros/ros.h>
#include "dog_nav_rviz_plugins/pose3d_tool.h"
#endif

namespace rviz
{
class StringProperty;
}

namespace dog_nav_rviz
{

class Goal3DTool : public Pose3DTool
{
  Q_OBJECT
public:
  Goal3DTool();
  void onInitialize() override;
  void activate() override;

protected:
  bool onPoseSet(double x, double y, double z, double yaw, rviz::ViewportMouseEvent&) override;

private Q_SLOTS:
  void updateTopic();

private:
  bool publishElevatorMark(double x, double y, double z, rviz::ViewportMouseEvent& event);
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  rviz::StringProperty* topic_property_;
};

}  // namespace dog_nav_rviz

#endif
