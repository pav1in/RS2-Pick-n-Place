#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <string>
#include <vector>
#include <utility>

class SpawnManager
{
public:
  SpawnManager(ros::NodeHandle& nh, const std::string& model_name);

  bool spawnBox(const std::string& instance_name, const geometry_msgs::Pose& pose);
  bool deleteBox(const std::string& instance_name);

  void spawnRandomBoxes(int count);
  const std::vector<std::pair<std::string, geometry_msgs::Pose>>& getSpawnedBoxes() const;

private:
  ros::ServiceClient spawn_client_;
  ros::ServiceClient delete_client_;
  std::string model_xml_;
  std::vector<std::pair<std::string, geometry_msgs::Pose>> spawned_boxes_;

  geometry_msgs::Pose generateRandomPose();
  bool isFarEnough(const geometry_msgs::Pose& a,
                   const geometry_msgs::Pose& b,
                   double min_dist = 0.1) const;
};
