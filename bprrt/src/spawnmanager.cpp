// File: src/spawnmanager.cpp

#include "bprrt/spawnmanager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <cmath>
#include <rclcpp/executors/single_threaded_executor.hpp>

using namespace std::chrono_literals;

namespace bprrt {

SpawnManager::SpawnManager(
  rclcpp::Node::SharedPtr node,
  const std::string & model_name)
: node_(node)
{
  // point at the *un*-namespaced factory services
  spawn_client_  = node_->create_client<ros_gz_interfaces::srv::SpawnEntity>("/spawn_entity");
  delete_client_ = node_->create_client<ros_gz_interfaces::srv::DeleteEntity>("/delete_entity");

  if (!spawn_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(node_->get_logger(), "Service '/spawn_entity' not available");
  }
  if (!delete_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(node_->get_logger(), "Service '/delete_entity' not available");
  }

  // Load the SDF from bprrt/models/<model_name>/model.sdf
  std::string pkg = ament_index_cpp::get_package_share_directory("bprrt");
  std::ifstream ifs(pkg + "/models/" + model_name + "/model.sdf");
  if (!ifs) {
    RCLCPP_ERROR(node_->get_logger(), "Could not open SDF model for [%s]", model_name.c_str());
  }
  std::stringstream ss;
  ss << ifs.rdbuf();
  model_xml_ = ss.str();

  std::srand(static_cast<unsigned>(std::time(nullptr)));
}

geometry_msgs::msg::Pose SpawnManager::generateRandomPose()
{
  geometry_msgs::msg::Pose p;
  p.position.x = 0.2 + double(std::rand())/(RAND_MAX/0.3);
  p.position.y = -0.3 + double(std::rand())/(RAND_MAX/0.6);
  p.position.z = 0.0;
  p.orientation.w = 1.0;
  return p;
}

bool SpawnManager::spawnBox(
  const std::string & name,
  const geometry_msgs::msg::Pose & pose)
{
  auto req = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
  req->entity_factory.name = name;
  req->entity_factory.sdf = model_xml_;
  req->entity_factory.allow_renaming = true;
  req->entity_factory.pose = pose;

  auto fut = spawn_client_->async_send_request(req);

  // spin this node in its own executor so we don't re-register it
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  auto ret = exec.spin_until_future_complete(fut, 5s);
  exec.remove_node(node_);
  
  if (ret == rclcpp::FutureReturnCode::SUCCESS && fut.get()->success) {
    RCLCPP_INFO(node_->get_logger(), "Spawned [%s]", name.c_str());
    spawned_boxes_.emplace_back(name, pose);
    return true;
  }
  RCLCPP_ERROR(node_->get_logger(), "Failed to spawn [%s]", name.c_str());
  return false;
}

bool SpawnManager::deleteBox(const std::string & name)
{
  auto req = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
  req->entity.name = name;
  req->entity.type = ros_gz_interfaces::msg::Entity::MODEL;

  auto fut = delete_client_->async_send_request(req);

  // again, local executor for just this node
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  auto ret = exec.spin_until_future_complete(fut, 5s);
  exec.remove_node(node_);
  
  if (ret == rclcpp::FutureReturnCode::SUCCESS && fut.get()->success) {
    RCLCPP_INFO(node_->get_logger(), "Deleted [%s]", name.c_str());
    return true;
  }
  RCLCPP_ERROR(node_->get_logger(), "Failed to delete [%s]", name.c_str());
  return false;
}

void SpawnManager::spawnRandomBoxes(int count)
{
  for (int i = 0; i < count; ++i) {
    geometry_msgs::msg::Pose p;
    bool ok = false;
    for (int attempt = 0; attempt < 50; ++attempt) {
      p = generateRandomPose();
      ok = true;
      for (auto & b : spawned_boxes_) {
        double dx = p.position.x - b.second.position.x;
        double dy = p.position.y - b.second.position.y;
        if (std::hypot(dx, dy) < 0.1) { ok = false; break; }
      }
      if (ok) break;
    }
    if (!ok) {
      RCLCPP_WARN(node_->get_logger(), "No free pose for box %d", i);
      continue;
    }
    auto now = node_->get_clock()->now().nanoseconds();
    std::string uname = "box_" + std::to_string(now);
    std::this_thread::sleep_for(10ms);
    spawnBox(uname, p);
  }
}

const std::vector<std::pair<std::string, geometry_msgs::msg::Pose>> &
SpawnManager::getSpawnedBoxes() const
{
  return spawned_boxes_;
}

}  // namespace bprrt
