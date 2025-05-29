// File: src/box_spawner_node.cpp

#include "bprrt/spawnmanager.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("box_spawner_node");

  // manager will wait for /spawn_entity service in its ctor
  auto manager = std::make_shared<bprrt::SpawnManager>(node, "simplebox");

  // Timer to fire once (after 1s) and spawn 5 boxes
  rclcpp::TimerBase::SharedPtr timer;
  timer = node->create_wall_timer(
    std::chrono::seconds(1),
    [manager, &timer]() {
      manager->spawnRandomBoxes(5);
      timer->cancel();  // only once
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
