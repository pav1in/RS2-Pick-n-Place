cmake_minimum_required(VERSION 2.8.3)
project(yolov8_detector_py)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
)

catkin_package(
  # No library or executables are built since this is a Python package.
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/yolov8_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
