#include "bprrt/boxtracker.hpp"

BoxTracker::BoxTracker()
: Node("box_tracker_node")
{
  sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
    "/gazebo/model_states", 10,
    std::bind(&BoxTracker::modelCallback, this, std::placeholders::_1));

  pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "/spawned_boxes", 1);
}

void BoxTracker::modelCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
  geometry_msgs::msg::PoseArray arr;
  arr.header.stamp = this->get_clock()->now();
  arr.header.frame_id = "world";
  arr.poses.clear();

  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    if (msg->name[i].rfind("box_", 0) == 0)
    {
      arr.poses.push_back(msg->pose[i]);
    }
  }

  pub_->publish(arr);
}
