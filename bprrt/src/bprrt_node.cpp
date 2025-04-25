#include "bprrt_node.h"
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/robot_state/conversions.h>  // robotStateToRobotStateMsg

namespace bprrt {

BPRRTPlannerNode::BPRRTPlannerNode(ros::NodeHandle& nh_global,
                                   ros::NodeHandle& nh_private)
  : move_group_("manipulator")
{
  // --- load planning parameters ---
  nh_private.param("bounds/x_min", bounds_low_.x(), -1.0);
  nh_private.param("bounds/y_min", bounds_low_.y(), -1.0);
  nh_private.param("bounds/z_min", bounds_low_.z(),  0.0);
  nh_private.param("bounds/x_max", bounds_high_.x(),  1.0);
  nh_private.param("bounds/y_max", bounds_high_.y(),  1.0);
  nh_private.param("bounds/z_max", bounds_high_.z(),  1.0);

  nh_private.param("step_size",       step_size_,       0.1);
  nh_private.param("max_iters",       max_iters_,       500);
  nh_private.param("goal_bias",       goal_bias_,       0.05);
  nh_private.param("neighbor_radius", neighbor_radius_, 0.5);

  // subscriber and pubs
  boxes_sub_     = nh_global.subscribe("/spawned_boxes", 1,
                      &BPRRTPlannerNode::onNewBoxes, this);
  path_pub_      = nh_global.advertise<geometry_msgs::PoseArray>(
                      "/bprrt_path", 1, true);
  cart_traj_pub_ = nh_global.advertise<moveit_msgs::DisplayTrajectory>(
                      "/bprrt_cartesian_trajectory", 1, true);

  ROS_INFO("[bprrt_node] initialized");
}

bool BPRRTPlannerNode::planTo(const Eigen::Vector3d& start_pt,
                              const Eigen::Vector3d& goal_pt,
                              geometry_msgs::PoseArray& out_path)
{
  ROS_INFO("🔄 planTo: constructing BP_RRTStar(start=(%.2f,%.2f,%.2f), goal=(%.2f,%.2f,%.2f))",
           start_pt.x(), start_pt.y(), start_pt.z(),
           goal_pt.x(),  goal_pt.y(),  goal_pt.z());

  BP_RRTStar planner(start_pt, goal_pt,
                     Bounds{bounds_low_, bounds_high_},
                     step_size_, max_iters_,
                     goal_bias_, neighbor_radius_);
  if (!planner.plan())
  {
    ROS_WARN("❌ BP_RRTStar::plan() failed");
    return false;
  }
  ROS_INFO("✅ BP_RRTStar::plan() succeeded");

  auto poses = planner.getPathMsg();
  out_path.header.stamp    = ros::Time::now();
  out_path.header.frame_id = "world";
  out_path.poses = std::move(poses);
  ROS_INFO("→ planTo: produced %zu waypoints", out_path.poses.size());
  return true;
}

bool BPRRTPlannerNode::planCartesian(const std::vector<geometry_msgs::Pose>& rrt_path,
                                     moveit_msgs::RobotTrajectory& out_traj)
{
  ROS_INFO("→ planCartesian start (got %zu RRT waypoints)", rrt_path.size());

  // 1) densify segments
  std::vector<geometry_msgs::Pose> dense;
  const int DIVS = 5;
  for (size_t i = 1; i < rrt_path.size(); ++i)
  {
    const auto &A = rrt_path[i-1], &B = rrt_path[i];
    dense.push_back(A);
    for (int k = 1; k < DIVS; ++k)
    {
      double t = double(k)/DIVS;
      geometry_msgs::Pose M;
      M.position.x = A.position.x + t*(B.position.x - A.position.x);
      M.position.y = A.position.y + t*(B.position.y - A.position.y);
      M.position.z = A.position.z + t*(B.position.z - A.position.z);
      M.orientation = A.orientation;
      dense.push_back(M);
    }
  }
  dense.push_back(rrt_path.back());
  ROS_INFO("   → planCartesian: densified to %zu waypoints", dense.size());

  // 2) compute collision-checked Cartesian
  moveit_msgs::RobotTrajectory cart;
  moveit_msgs::MoveItErrorCodes ec;
  double frac = move_group_.computeCartesianPath(
    dense,     // waypoints
    0.01,      // eef_step
    0.0,       // jump_threshold (ignored)
    cart,
    /*avoid_collisions=*/true,
    &ec);

  ROS_INFO("   → computeCartesianPath → frac=%.1f%%, err=%d", frac*100.0, ec.val);
  if (ec.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_WARN("❌ Cartesian only %.1f%% achievable", frac*100.0);
    return false;
  }

  out_traj = std::move(cart);
  ROS_INFO("✅ planCartesian succeeded: %zu trajectory points",
           out_traj.joint_trajectory.points.size());
  return true;
}

void BPRRTPlannerNode::onNewBoxes(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  // Option B: only act on newly stamped messages
  if (msg->header.stamp == last_boxes_stamp_)
  {
    ROS_DEBUG("onNewBoxes: same stamp, skipping");
    return;
  }
  last_boxes_stamp_ = msg->header.stamp;

  ROS_INFO("🔄 onNewBoxes: planning raw RRT*");
  auto ee = move_group_.getCurrentPose();
  Eigen::Vector3d start_pt{
    ee.pose.position.x,
    ee.pose.position.y,
    ee.pose.position.z
  };

  for (auto& goal_pose : msg->poses)
  {
    // lift 10 cm above box
    Eigen::Vector3d goal_pt{
      goal_pose.position.x,
      goal_pose.position.y,
      goal_pose.position.z + 0.10
    };

    // 1) RRT*
    geometry_msgs::PoseArray rrt_path;
    if (!planTo(start_pt, goal_pt, rrt_path))
    {
      ROS_WARN("RRT* failed for (%.2f,%.2f,%.2f)",
               goal_pt.x(), goal_pt.y(), goal_pt.z());
      continue;
    }
    path_pub_.publish(rrt_path);

    // 2) Cartesian
    ROS_INFO("🔄 onNewBoxes: invoking planCartesian() on %zu waypoints",
             rrt_path.poses.size());
    std::vector<geometry_msgs::Pose> waypoints(rrt_path.poses.begin(),
                                              rrt_path.poses.end());

    moveit_msgs::RobotTrajectory cart;
    if (planCartesian(waypoints, cart))
    {
      // convert current state → message
      moveit_msgs::RobotState start_msg;
      auto state = move_group_.getCurrentState();
      moveit::core::robotStateToRobotStateMsg(*state, start_msg);

      moveit_msgs::DisplayTrajectory disp;
      disp.trajectory_start = start_msg;
      disp.trajectory.push_back(cart);
      cart_traj_pub_.publish(disp);

      ROS_INFO("Published Cartesian trajectory (%zu points)",
               cart.joint_trajectory.points.size());
    }
    else
    {
      ROS_WARN("planCartesian() returned FAILURE");
    }
  }
}

void BPRRTPlannerNode::spin()
{
  ros::spin();
}

}  // namespace bprrt

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bprrt_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle nh_global, nh_private("~");
  bprrt::BPRRTPlannerNode node(nh_global, nh_private);
  ros::waitForShutdown();
  return 0;
}
