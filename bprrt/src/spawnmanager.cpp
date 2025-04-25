#include "spawnmanager.h"
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <ctime>
#include <ros/package.h>

SpawnManager::SpawnManager(ros::NodeHandle& nh, const std::string& model_name)
{
  spawn_client_  = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
  delete_client_ = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

  auto pkg_path = ros::package::getPath("bprrt");
  std::ifstream ifs(pkg_path + "/models/" + model_name + "/model.sdf");
  std::stringstream ss;
  ss << ifs.rdbuf();
  model_xml_ = ss.str();

  std::srand(std::time(nullptr));
}

geometry_msgs::Pose SpawnManager::generateRandomPose()
{
  geometry_msgs::Pose p;
  p.position.x    = 0.2 + static_cast<double>(rand()) / (RAND_MAX / 0.3);
  p.position.y    = -0.3 + static_cast<double>(rand()) / (RAND_MAX / 0.6);
  p.position.z    = 0.0;
  p.orientation.w = 1.0;
  return p;
}

bool SpawnManager::spawnBox(const std::string& name, const geometry_msgs::Pose& pose)
{
  gazebo_msgs::SpawnModel srv;
  srv.request.model_name      = name;
  srv.request.model_xml       = model_xml_;
  srv.request.robot_namespace = "";
  srv.request.initial_pose    = pose;
  srv.request.reference_frame = "world";

  if (spawn_client_.call(srv) && srv.response.success)
  {
    ROS_INFO("Spawned box [%s]", name.c_str());
    spawned_boxes_.emplace_back(name, pose);
    return true;
  }
  ROS_ERROR("Failed to spawn box [%s]", name.c_str());
  return false;
}

bool SpawnManager::deleteBox(const std::string& name)
{
  gazebo_msgs::DeleteModel srv;
  srv.request.model_name = name;
  return delete_client_.call(srv) && srv.response.success;
}

void SpawnManager::spawnRandomBoxes(int count)
{
  for (int i = 0; i < count; ++i)
  {
    geometry_msgs::Pose p;
    bool ok = false;
    for (int attempt = 0; attempt < 50; ++attempt)
    {
      p  = generateRandomPose();
      ok = true;
      for (auto& b : spawned_boxes_)
      {
        if (!isFarEnough(p, b.second))
        {
          ok = false;
          break;
        }
      }
      if (ok) break;
    }
    if (!ok)
    {
      ROS_WARN("Could not find a free pose for box %d", i);
      continue;
    }
    auto name = "box_" + std::to_string(ros::Time::now().toNSec());
    ros::Duration(0.01).sleep();
    spawnBox(name, p);
  }
}

bool SpawnManager::isFarEnough(const geometry_msgs::Pose& a,
                               const geometry_msgs::Pose& b,
                               double min_dist) const
{
  double dx = a.position.x - b.position.x;
  double dy = a.position.y - b.position.y;
  return (std::sqrt(dx*dx + dy*dy) >= min_dist);
}

const std::vector<std::pair<std::string, geometry_msgs::Pose>>&
SpawnManager::getSpawnedBoxes() const
{
  return spawned_boxes_;
}
