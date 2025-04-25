#ifndef BPRRT_NODE_H
#define BPRRT_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "bp_rrt_star_lib.h"
#include <memory>
#include <vector>
#include <cmath>

namespace bprrt {

class BPRRTPlannerNode
{
public:
  BPRRTPlannerNode(ros::NodeHandle& nh_global,
                   ros::NodeHandle& nh_private);
  void spin();

private:
  // Core planners
  bool planTo(const Eigen::Vector3d& start_pt,
              const Eigen::Vector3d& goal_pt,
              geometry_msgs::PoseArray& out_path);
  bool planCartesian(const std::vector<geometry_msgs::Pose>& rrt_path,
                     moveit_msgs::RobotTrajectory& out_traj);
                       /// Time-parameterize a RobotTrajectory in place.
  bool timeParameterizeTrajectory(moveit_msgs::RobotTrajectory &traj);

  /// Given a time-stamped RobotTrajectory, wrap & execute it.
  bool executeTrajectory(const moveit_msgs::RobotTrajectory &traj);

  // Incoming‐message buffer + timer
  void onBoxMessage(const geometry_msgs::PoseArray::ConstPtr& msg);
  void onPlanTimer(const ros::TimerEvent& ev);

  // solve IK on each RRT pose, build a joint-space RobotTrajectory
  bool computeJointTrajectory(const std::vector<geometry_msgs::Pose>& waypoints,
    moveit_msgs::RobotTrajectory& traj_msg);


  // ROS interfaces
  ros::Subscriber               boxes_sub_;
  ros::Publisher                path_pub_;
  ros::Publisher                cart_traj_pub_;
  ros::Timer                    plan_timer_;

  // Buffered data from /spawned_boxes
  geometry_msgs::PoseArray      latest_boxes_;
  ros::Time                     latest_boxes_stamp_;

  // Planning parameters
  double                        plan_rate_;        // Hz
  Eigen::Vector3d               bounds_low_, bounds_high_;
  double                        step_size_;
  int                           max_iters_;
  double                        goal_bias_;
  double                        neighbor_radius_;
  bool                    new_data_{false};
  std::vector<geometry_msgs::Pose> buf_poses_;
  double                  replan_dist_thresh_{0.01};  // 1 cm
  // MoveIt!
  moveit::planning_interface::MoveGroupInterface move_group_;

};

}  // namespace bprrt

#endif  // BPRRT_NODE_H
