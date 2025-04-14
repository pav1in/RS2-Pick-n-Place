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
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs gazebo_ros gazebo_msgs moveit_ros_planning_interface tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Existing targets:
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

add_library(bp_rrt_planner
  src/bp_rrt_planner.cpp
)
target_link_libraries(bp_rrt_planner ${catkin_LIBRARIES})

add_library(box_tracker
  src/box_tracker.cpp
)
target_link_libraries(box_tracker ${catkin_LIBRARIES})

add_executable(bp_rrt_planner_node
  src/bp_rrt_planner_node.cpp
)
target_link_libraries(bp_rrt_planner_node
  bp_rrt_planner
  box_tracker
  ${catkin_LIBRARIES}
)

add_library(rrt_star_planner_lib
  src/rrt_star_planner_lib.cpp
)
target_link_libraries(rrt_star_planner_lib ${catkin_LIBRARIES})

# Add the MoveIt plugin target wrapping the RRT* library.
add_library(rrt_star_planner_plugin
  src/rrt_star_planner_plugin.cpp
)
install(FILES config/rrt_star_planner_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

target_link_libraries(rrt_star_planner_plugin
  rrt_star_planner_lib
  box_tracker
  ${catkin_LIBRARIES}
)
