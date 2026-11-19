cmake_minimum_required(VERSION 2.8.3)
project(scan_upstream_search)

set(CMAKE_BUILD_TYPE "Release")

ADD_COMPILE_OPTIONS(-std=c++17 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g0")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  scan_upstream_env
  cv_bridge
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES scan_upstream_search
 CATKIN_DEPENDS scan_upstream_env
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( scan_upstream_search 
    src/dyn_a_star.cpp
    )
target_link_libraries( scan_upstream_search
    ${catkin_LIBRARIES} 
    )  
