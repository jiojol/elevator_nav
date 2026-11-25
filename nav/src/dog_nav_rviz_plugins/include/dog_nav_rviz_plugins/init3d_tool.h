#ifndef DOG_NAV_RVIZ_INIT3D_TOOL_H
#define DOG_NAV_RVIZ_INIT3D_TOOL_H

#ifndef Q_MOC_RUN
#include <QObject>
#include <ros/ros.h>
#include "dog_nav_rviz_plugins/pose3d_tool.h"
#endif

namespace rviz
{
class FloatProperty;
class StringProperty;
}

namespace dog_nav_rviz
{

class Init3DTool : public Pose3DTool
{
  Q_OBJECT
public:
  Init3DTool();
  void onInitialize() override;

protected:
  bool onPoseSet(double x, double y, double z, double yaw, rviz::ViewportMouseEvent&) override;

private Q_SLOTS:
  void updateTopic();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  rviz::StringProperty* topic_property_;
  rviz::FloatProperty* std_dev_x_;
  rviz::FloatProperty* std_dev_y_;
  rviz::FloatProperty* std_dev_z_;
  rviz::FloatProperty* std_dev_theta_;
};

}  // namespace dog_nav_rviz

#endif
