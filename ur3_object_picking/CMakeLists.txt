cmake_minimum_required(VERSION 3.0.2)
project(ur3_object_picking)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  gazebo_ros
  gazebo_msgs
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs gazebo_ros gazebo_msgs
                 moveit_ros_planning_interface tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# === Executables ===
add_executable(spawn_demo_node
  src/spawn_demo_node.cpp
  src/spawn_manager.cpp
)
target_link_libraries(spawn_demo_node ${catkin_LIBRARIES})

add_executable(ur3_planner_node
  src/ur3_planner_node.cpp
  src/spawn_manager.cpp
)
target_link_libraries(ur3_planner_node ${catkin_LIBRARIES})

# === Optional support classes ===
add_library(box_tracker
  src/box_tracker.cpp
)
target_link_libraries(box_tracker ${catkin_LIBRARIES})

add_library(bp_rrt_planner  # You can remove this if not needed for other logic
  src/bp_rrt_planner.cpp
)
target_link_libraries(bp_rrt_planner ${catkin_LIBRARIES})

# === Installation ===
install(TARGETS
  spawn_demo_node
  ur3_planner_node
  box_tracker
  bp_rrt_planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch config models rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
