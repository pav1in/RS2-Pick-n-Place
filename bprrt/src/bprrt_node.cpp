#include "bprrt_node.h"

namespace bprrt
{

BPRRTPlannerNode::BPRRTPlannerNode(ros::NodeHandle nh)
  : nh_(std::move(nh))
{
  // topics
  sub_start_ = nh_.subscribe("start", 1, &BPRRTPlannerNode::startCb, this);
  sub_goal_  = nh_.subscribe("goal",  1, &BPRRTPlannerNode::goalCb,  this);
  pub_path_  = nh_.advertise<geometry_msgs::PoseArray>("bprrt_path", 1);

  // load your params
  nh_.param("bounds/x_min", bounds_.low.x(), -10.0);
  nh_.param("bounds/y_min", bounds_.low.y(), -10.0);
  nh_.param("bounds/z_min", bounds_.low.z(), -10.0);
  nh_.param("bounds/x_max", bounds_.high.x(),  10.0);
  nh_.param("bounds/y_max", bounds_.high.y(),  10.0);
  nh_.param("bounds/z_max", bounds_.high.z(),  10.0);

  nh_.param("step_size",       step_size_,       0.5);
  nh_.param("max_iters",       max_iters_,       500);
  nh_.param("goal_bias",       goal_bias_,       0.1);
  nh_.param("neighbor_radius", neighbor_radius_, 1.0);
}

void BPRRTPlannerNode::startCb(const geometry_msgs::PoseStamped& p)
{
  start_pt_ << p.pose.position.x,
               p.pose.position.y,
               p.pose.position.z;
  have_start_ = true;
}

void BPRRTPlannerNode::goalCb(const geometry_msgs::PoseStamped& p)
{
  goal_pt_ << p.pose.position.x,
              p.pose.position.y,
              p.pose.position.z;
  have_goal_ = true;
}

void BPRRTPlannerNode::planAndPublish()
{
  // instantiate a fresh planner with the up‑to‑date start, goal and params
  BP_RRTStar planner(
    start_pt_,
    goal_pt_,
    bounds_,
    step_size_,
    max_iters_,
    goal_bias_,
    neighbor_radius_);

  if (!planner.plan())
  {
    ROS_WARN("BP-RRT* failed to find a path");
    return;
  }

  auto poses = planner.getPathMsg();
  geometry_msgs::PoseArray arr;
  arr.header.stamp    = ros::Time::now();
  arr.header.frame_id = "world";
  arr.poses           = std::move(poses);
  pub_path_.publish(arr);
}

void BPRRTPlannerNode::spin()
{
  ros::Rate rate(1.0);
  while (ros::ok())
  {
    ros::spinOnce();
    if (have_start_ && have_goal_)
    {
      planAndPublish();
      have_start_ = have_goal_ = false;
    }
    rate.sleep();
  }
}

} // namespace bprrt

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bprrt_node");
  bprrt::BPRRTPlannerNode node(ros::NodeHandle("~"));
  node.spin();
  return 0;
}
