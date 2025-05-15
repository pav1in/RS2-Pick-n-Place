#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "bprrt/spawnmanager.hpp"  // make sure this path matches your include folder

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("box_spawner");

  // Fully-qualified SpawnManager in the bprrt namespace:
  bprrt::SpawnManager mgr(node, "simple_box");

  // Spawn one box (you can increase the count or loop as desired)
  mgr.spawnRandomBoxes(1);

  // Keep the node alive for any callbacks (if needed)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
