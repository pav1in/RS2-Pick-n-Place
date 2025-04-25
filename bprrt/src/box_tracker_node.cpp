#include <ros/ros.h>
#include "boxtracker.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "box_tracker_node");
  ros::NodeHandle nh;

  BoxTracker tracker(nh);
  ros::spin();
  return 0;
}
