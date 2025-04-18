#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include "bp_rrt_star_lib.h"   // brings in bprrt::Bounds

namespace bprrt
{

class BPRRTPlannerNode
{
public:
  // pass NodeHandle by value (so we can give a nice default) 
  explicit BPRRTPlannerNode(ros::NodeHandle nh = ros::NodeHandle("~"));

  void spin();

private:
  void startCb(const geometry_msgs::PoseStamped& p);
  void goalCb (const geometry_msgs::PoseStamped& p);
  void planAndPublish();

  ros::NodeHandle    nh_;
  ros::Subscriber    sub_start_, sub_goal_;
  ros::Publisher     pub_path_;

  Eigen::Vector3d    start_pt_{0,0,0}, goal_pt_{0,0,0};
  bool               have_start_{false}, have_goal_{false};

  // loaded from params/bp_rrt_params.yaml
  Bounds             bounds_;
  double             step_size_;
  int                max_iters_;
  double             goal_bias_;
  double             neighbor_radius_;
};

} // namespace bprrt
