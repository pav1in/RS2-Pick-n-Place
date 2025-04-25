cmake_minimum_required(VERSION 2.8.3)
project(bprrt)

## 1 Compiler flags
add_compile_options(-std=c++17 -Wall -Wextra -Wno-unused-parameter -fsanitize=address -fno-omit-frame-pointer) 
add_link_options(-fsanitize=address)

## 2 Find ROS, MoveIt!, Eigen
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  gazebo_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  pluginlib
)
find_package(Eigen3 REQUIRED)

## 3 Find LibTorch (CPU‐only)
find_package(Torch REQUIRED PATHS $ENV{TORCH_HOME}/share/cmake/Torch)

## 4 Catkin meta‐package exports
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES spawnmanager boxtracker bp_rrt_star_lib
  CATKIN_DEPENDS
    roscpp std_msgs geometry_msgs gazebo_msgs
    tf2 tf2_ros tf2_geometry_msgs
    moveit_core moveit_ros_planning_interface pluginlib
)

## 5 Include paths
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
)

## 6 spawnmanager library
add_library(spawnmanager
  src/spawnmanager.cpp
)
target_link_libraries(spawnmanager
  ${catkin_LIBRARIES}
)

## 7 boxtracker library
add_library(boxtracker
  src/boxtracker.cpp
)
target_include_directories(boxtracker PUBLIC include)
target_link_libraries(boxtracker
  ${catkin_LIBRARIES}
)

add_executable(box_tracker_node src/box_tracker_node.cpp)
target_link_libraries(box_tracker_node
  boxtracker           
  ${catkin_LIBRARIES}
)
install(TARGETS box_tracker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 8 bp_rrt_star core library
add_library(bp_rrt_star_lib
  src/bp_rrt_star_lib.cpp
)
target_link_libraries(bp_rrt_star_lib
  ${catkin_LIBRARIES}
  ${TORCH_LIBRARIES}
)

## 9 box_spawner_node
add_executable(box_spawner_node
  src/box_spawner_node.cpp
)
target_link_libraries(box_spawner_node
  spawnmanager
  ${catkin_LIBRARIES}
)

## 10 bprrt_node
add_executable(bprrt_node
  src/bprrt_node.cpp
)
target_link_libraries(bprrt_node
  spawnmanager
  boxtracker
  bp_rrt_star_lib
  ${catkin_LIBRARIES}
  ${TORCH_LIBRARIES}
)

## 11 Install
install(TARGETS
    spawnmanager
    boxtracker
    bp_rrt_star_lib
    box_spawner_node
    bprrt_node
    
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
