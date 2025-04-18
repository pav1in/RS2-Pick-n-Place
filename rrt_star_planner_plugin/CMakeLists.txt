cmake_minimum_required(VERSION 3.0.2)
project(rrt_star_planner_plugin)

add_compile_options(-std=c++14)

## === Find required packages ===
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_msgs
  geometry_msgs
)

## === Declare catkin package ===
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pluginlib moveit_core moveit_ros_planning moveit_msgs geometry_msgs
)

## === Include headers ===
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## === Add plugin library ===
add_library(${PROJECT_NAME}
  src/rrt_star_planner_plugin.cpp
  src/rrt_star_planner_lib.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## === Install plugin .so library ===
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## === Install plugin description XML ===
install(FILES config/rrt_star_planner_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

## === Write plugin XML to devel space so pluginlib can see it at runtime ===
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/config)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config/rrt_star_planner_plugin_description.xml
  ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/config/rrt_star_planner_plugin_description.xml
  COPYONLY
)
