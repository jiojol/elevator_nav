cmake_minimum_required(VERSION 3.0.2)
project(dog_nav_bringup)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2_ros
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  pcl_ros
  message_filters
  fast_gicp
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(reference_path_transform src/reference_path_transform.cpp)
target_link_libraries(reference_path_transform ${catkin_LIBRARIES})

add_executable(fastlio_map_localizer src/fastlio_map_localizer.cpp)
target_link_libraries(fastlio_map_localizer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(elevator_lio_adapter src/elevator_lio_adapter.cpp)
target_link_libraries(elevator_lio_adapter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Optional hardware adapter; mapping/planning-only builds work without SDK2.
find_path(UNITREE_NAV_INCLUDE unitree/robot/go2/sport/sport_client.hpp
  HINTS "$ENV{DOG_NAV_UNITREE_SDK2_ROOT}/include"
        "$ENV{HOME}/unitree_sdk2/include")
if(UNITREE_NAV_INCLUDE)
  get_filename_component(UNITREE_NAV_ROOT "${UNITREE_NAV_INCLUDE}" DIRECTORY)
  set(UNITREE_NAV_LIB "${UNITREE_NAV_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_sdk2.a")
  set(UNITREE_NAV_DDS "${UNITREE_NAV_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")
  if(EXISTS "${UNITREE_NAV_LIB}")
    add_executable(unitree_nav_bridge src/unitree_nav_bridge.cpp)
    set_property(TARGET unitree_nav_bridge PROPERTY CXX_STANDARD 17)
    target_include_directories(unitree_nav_bridge PRIVATE "${UNITREE_NAV_INCLUDE}"
      "${UNITREE_NAV_ROOT}/thirdparty/include" "${UNITREE_NAV_ROOT}/thirdparty/include/ddscxx")
    target_link_libraries(unitree_nav_bridge ${catkin_LIBRARIES} "${UNITREE_NAV_LIB}"
      "${UNITREE_NAV_DDS}/libddscxx.so" "${UNITREE_NAV_DDS}/libddsc.so" pthread)
  endif()
endif()

catkin_install_python(PROGRAMS
  scripts/fastlio_nav.py
  scripts/scan_command_gate.py
  scripts/elevator_mission_manager.py
  scripts/elevator_door_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES scripts/start_alignment_policy.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
