cmake_minimum_required(VERSION 3.0.2)
project(fastlio)

# GTSAM 新版本使用 C++17（std::optional / std::variant 等），这里统一切到 C++17
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")


find_package(catkin REQUIRED COMPONENTS
  tf2
  tf2_ros
  roscpp
  rosbag
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  livox_ros_driver2
  message_generation
  eigen_conversions
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Prefer an explicit GTSAM, then the local 4.2 build, then the aarch64
# GTSAM 4.1.1 already built for this robot.
get_filename_component(FASTLIO_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(FASTLIO_GTSAM_HINTS
  "$ENV{GTSAM_DIR}"
  "$ENV{HOME}/.local/gtsam-4.2.0/lib/cmake/GTSAM"
  "${FASTLIO_REPO_ROOT}/third_party/gtsam/lib/cmake/GTSAM")
find_package(GTSAM QUIET HINTS ${FASTLIO_GTSAM_HINTS})
if(GTSAM_FOUND)
  message(STATUS "GTSAM found: loop closure backend enabled")
  set(FASTLIO_GTSAM_DEFINITION FASTLIO_WITH_GTSAM=1)
  set(FASTLIO_GTSAM_LIBRARIES gtsam)
  get_filename_component(FASTLIO_GTSAM_PREFIX "${GTSAM_DIR}/../../.." ABSOLUTE)
  set(FASTLIO_GTSAM_LIBRARY_DIR "${FASTLIO_GTSAM_PREFIX}/lib")
  catkin_add_env_hooks(fastlio_gtsam
    SHELLS sh
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
else()
  message(WARNING "GTSAM not found: building map_builder_node without loop closure backend")
  set(FASTLIO_GTSAM_DEFINITION FASTLIO_WITH_GTSAM=0)
  set(FASTLIO_GTSAM_LIBRARIES "")
endif()



add_service_files(FILES SaveMap.srv)



generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS
    tf2 tf2_ros roscpp rosbag std_msgs nav_msgs sensor_msgs geometry_msgs
    pcl_ros livox_ros_driver2 message_runtime eigen_conversions
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

set(COMMON_SRC src/commons.cpp)
set(KDTREE_SRC include/ikd-Tree/ikd_Tree.cpp)
set(LIOBUILDER_SRC_LIST src/lio_builder/imu_processor.cpp src/lio_builder/lio_builder.cpp)
set(ICPLOCALIZER_SRC src/localizer/icp_localizer.cpp)
set(SRC_LIST ${COMMON_SRC} ${KDTREE_SRC} ${LIOBUILDER_SRC_LIST} ${ICPLOCALIZER_SRC})



add_executable(map_builder_node src/map_builder_node.cpp ${SRC_LIST})
add_dependencies(map_builder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_definitions(map_builder_node PRIVATE ${FASTLIO_GTSAM_DEFINITION})

target_link_libraries(map_builder_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${FASTLIO_GTSAM_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/save_map.py
  scripts/evaluate_tum_closure.py
  scripts/extract_nav_bag_traj_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
