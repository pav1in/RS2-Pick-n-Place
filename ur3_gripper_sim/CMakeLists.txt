cmake_minimum_required(VERSION 3.0.2)
project(ur3_gripper_sim)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros_control
  joint_state_publisher
  robot_state_publisher
  rospy
  std_msgs
  urdf
  xacro
)


catkin_package(
  CATKIN_DEPENDS controller_manager gazebo_ros_control joint_state_publisher robot_state_publisher rospy std_msgs urdf xacro
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
