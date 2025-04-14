#include "ur3_object_picking/spawn_manager.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spawn_demo_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    SpawnManager manager(nh, "simple_box");
    manager.spawnRandomBoxes(5);

    ros::waitForShutdown();
    return 0;
}
