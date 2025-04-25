#include "boxtracker.h"

BoxTracker::BoxTracker(ros::NodeHandle& nh)
{
  sub_ = nh.subscribe("/gazebo/model_states", 10,
                     &BoxTracker::modelCallback, this);
  pub_ = nh.advertise<geometry_msgs::PoseArray>("/spawned_boxes", 1, true);
}

void BoxTracker::modelCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  geometry_msgs::PoseArray arr;
  arr.header.stamp    = ros::Time::now();
  arr.header.frame_id = "world";
  arr.poses.clear();

  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    
    if (msg->name[i].rfind("box_", 0) == 0)
    {
      
      arr.poses.push_back(msg->pose[i]);
    }
  }

  pub_.publish(arr);
}
