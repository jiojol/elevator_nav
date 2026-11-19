cmake_minimum_required(VERSION 3.0.2)
project(map_for_plan)

find_package(catkin REQUIRED COMPONENTS rospy nav_msgs sensor_msgs geometry_msgs visualization_msgs)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/tum_to_plan_map.py
  scripts/segmented_tum_to_plan_map.py
  scripts/visualize_plan_map.py
  scripts/global_planner.py
  scripts/plan_graph.py
  scripts/path_smoothing.py
  scripts/plan_io.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES scripts/cloud_contract.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
