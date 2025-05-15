cmake_minimum_required(VERSION 3.10)
project(bprrt)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find all the dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# Include headers
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# --- Main Planner Node ---
add_executable(bprrt_node
  src/bprrt_node.cpp
  src/bp_rrt_star_lib.cpp
)
target_include_directories(bprrt_node PUBLIC include)
ament_target_dependencies(bprrt_node
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  Eigen3
  tf2
  tf2_geometry_msgs
)

# --- Box Tracker Node ---
add_executable(box_tracker_node
  src/box_tracker_node.cpp
  src/boxtracker.cpp
)
target_include_directories(box_tracker_node PUBLIC include)
ament_target_dependencies(box_tracker_node
  rclcpp
  gazebo_msgs
  geometry_msgs
)

# --- Box Spawner Node ---
add_executable(box_spawner_node
  src/box_spawner_node.cpp
  src/spawnmanager.cpp
)
target_include_directories(box_spawner_node PUBLIC include)
ament_target_dependencies(box_spawner_node
  rclcpp
  gazebo_msgs
  geometry_msgs
  ament_index_cpp
  ros_gz_interfaces
)

# Install all executables
install(TARGETS
  bprrt_node
  box_tracker_node
  box_spawner_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
  FILES_MATCHING PATTERN "*.sdf"
)
# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
