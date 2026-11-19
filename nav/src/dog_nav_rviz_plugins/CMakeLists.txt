cmake_minimum_required(VERSION 3.0.2)
project(dog_nav_rviz_plugins)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

find_package(catkin REQUIRED COMPONENTS
  rviz
  roscpp
  geometry_msgs
  pluginlib
  tf2
)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package(
  CATKIN_DEPENDS rviz roscpp geometry_msgs pluginlib tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/pose3d_tool.cpp
  src/goal3d_tool.cpp
  src/init3d_tool.cpp
  include/dog_nav_rviz_plugins/goal3d_tool.h
  include/dog_nav_rviz_plugins/init3d_tool.h
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
