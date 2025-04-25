#include <ros/ros.h>
#include "spawnmanager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "box_spawner_node");
  ros::NodeHandle nh("~");
  SpawnManager mgr(nh, "simple_box");
  mgr.spawnRandomBoxes(1);
  ros::spin();
  return 0;
}
