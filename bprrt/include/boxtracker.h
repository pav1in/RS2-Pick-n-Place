#pragma once
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseArray.h>
#include <map>
#include <string>

class BoxTracker
{
public:
  BoxTracker(ros::NodeHandle& nh);

private:
  void modelCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

  ros::Subscriber sub_;
  ros::Publisher  pub_;
  std::map<std::string, geometry_msgs::Pose> boxes_;
};
