cmake_minimum_required(VERSION 3.0.2)
project(rrt_star_planner_plugin)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pluginlib moveit_core moveit_ros_planning moveit_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/rrt_star_planner_plugin.cpp
  src/rrt_star_planner_lib.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES config/rrt_star_planner_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
