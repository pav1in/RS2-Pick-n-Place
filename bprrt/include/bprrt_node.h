#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <Eigen/Geometry>
#include "bp_rrt_star_lib.h"

namespace bprrt {

class BPRRTPlannerNode {
public:
  explicit BPRRTPlannerNode(ros::NodeHandle& nh_global,
                            ros::NodeHandle& nh_private);
  void spin();

private:
  void onNewBoxes(const geometry_msgs::PoseArray::ConstPtr& msg);
  bool planTo(const Eigen::Vector3d& start_pt,
              const Eigen::Vector3d& goal_pt,
              geometry_msgs::PoseArray& out_path);
  bool planCartesian(const std::vector<geometry_msgs::Pose>& rrt_path,
                     moveit_msgs::RobotTrajectory& out_traj);

  // subscribers & publishers
  ros::Subscriber boxes_sub_;
  ros::Time       last_boxes_stamp_{0,0};
  ros::Publisher  path_pub_, cart_traj_pub_;

  // MoveIt interface
  moveit::planning_interface::MoveGroupInterface move_group_;

  // planning parameters
  Eigen::Vector3d bounds_low_, bounds_high_;
  double step_size_;
  int    max_iters_;
  double goal_bias_, neighbor_radius_;
};

} // namespace bprrt
