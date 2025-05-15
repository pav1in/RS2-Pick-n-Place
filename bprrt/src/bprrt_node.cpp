#include "bprrt/bprrt_node.hpp"
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace bprrt {

BPRRTPlannerNode::BPRRTPlannerNode()
: Node("bprrt_node"),
  move_group_(shared_from_this(), "manipulator")
{
  // Declare parameters
  this->declare_parameter("bounds.x_min", -1.0);
  this->declare_parameter("bounds.y_min", -1.0);
  this->declare_parameter("bounds.z_min",  0.0);
  this->declare_parameter("bounds.x_max",  1.0);
  this->declare_parameter("bounds.y_max",  1.0);
  this->declare_parameter("bounds.z_max",  1.0);
  this->declare_parameter("step_size",       0.1);
  this->declare_parameter("max_iters",       500);
  this->declare_parameter("goal_bias",       0.05);
  this->declare_parameter("neighbor_radius", 0.5);
  this->declare_parameter("plan_rate",       1.0);

  // Load parameters
  this->get_parameter("bounds.x_min", bounds_low_.x());
  this->get_parameter("bounds.y_min", bounds_low_.y());
  this->get_parameter("bounds.z_min", bounds_low_.z());
  this->get_parameter("bounds.x_max", bounds_high_.x());
  this->get_parameter("bounds.y_max", bounds_high_.y());
  this->get_parameter("bounds.z_max", bounds_high_.z());
  this->get_parameter("step_size", step_size_);
  this->get_parameter("max_iters", max_iters_);
  this->get_parameter("goal_bias", goal_bias_);
  this->get_parameter("neighbor_radius", neighbor_radius_);
  this->get_parameter("plan_rate", plan_rate_);

  RCLCPP_INFO(this->get_logger(), "Planner initialized: max_iters = %d", max_iters_);

  boxes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/spawned_boxes", 10,
    std::bind(&BPRRTPlannerNode::onBoxMessage, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "/bprrt_path", 1);

  cart_traj_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    "/move_group/display_planned_path", 1);

  plan_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / plan_rate_),
    std::bind(&BPRRTPlannerNode::onPlanTimer, this));
}

void BPRRTPlannerNode::onBoxMessage(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (msg->poses.size() != buf_poses_.size())
  {
    buf_poses_ = msg->poses;
    new_data_  = true;
    return;
  }

  for (size_t i = 0; i < msg->poses.size(); ++i)
  {
    const auto &A = buf_poses_[i].position;
    const auto &B = msg->poses[i].position;
    double dx = A.x - B.x, dy = A.y - B.y, dz = A.z - B.z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > replan_dist_thresh_)
    {
      buf_poses_ = msg->poses;
      new_data_  = true;
      return;
    }
  }
}

void BPRRTPlannerNode::onPlanTimer()
{
  if (!new_data_) return;
  new_data_ = false;

  RCLCPP_INFO(this->get_logger(), "Planning triggered");

  auto ee = move_group_.getCurrentPose();
  Eigen::Vector3d start_pt{
    ee.pose.position.x,
    ee.pose.position.y,
    ee.pose.position.z
  };

  if (buf_poses_.empty())
  {
    RCLCPP_WARN(this->get_logger(), "No boxes in buffer");
    return;
  }

  const double tool_offset = 0.10;
  double box_z = buf_poses_[0].position.z;
  double min_z = box_z + tool_offset;

  Eigen::Vector3d goal_pt{
    buf_poses_[0].position.x,
    buf_poses_[0].position.y,
    min_z
  };

  geometry_msgs::msg::PoseArray rrt_path;
  if (!planTo(start_pt, goal_pt, rrt_path))
  {
    RCLCPP_WARN(this->get_logger(), "planTo failed");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Publishing %zu RRT waypoints", rrt_path.poses.size());
  path_pub_->publish(rrt_path);

  tf2::Quaternion down_q;
  down_q.setRPY(M_PI_2, 0.0, 0.0);
  down_q.normalize();
  geometry_msgs::msg::Quaternion down_ori = tf2::toMsg(down_q);

  for (auto &wp : rrt_path.poses)
  {
    wp.position.z  = std::max(wp.position.z, min_z);
    wp.orientation = down_ori;
  }

  std::vector<geometry_msgs::msg::Pose> waypoints(
    rrt_path.poses.begin(), rrt_path.poses.end());

  moveit_msgs::msg::RobotTrajectory joint_traj;
  if (!computeJointTrajectory(waypoints, joint_traj) ||
      !timeParameterizeTrajectory(joint_traj))
  {
    RCLCPP_WARN(this->get_logger(), "Trajectory preparation failed");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Executing trajectory");
  executeTrajectory(joint_traj);
}

bool BPRRTPlannerNode::planTo(const Eigen::Vector3d& start_pt,
  const Eigen::Vector3d& goal_pt,
  geometry_msgs::msg::PoseArray& out_path)
{
RCLCPP_INFO(this->get_logger(),
"planTo: BP_RRTStar(start=(%.2f,%.2f,%.2f), goal=(%.2f,%.2f,%.2f))",
start_pt.x(), start_pt.y(), start_pt.z(),
goal_pt.x(), goal_pt.y(), goal_pt.z());

auto planner = std::make_unique<BP_RRTStar>(
start_pt, goal_pt,
Bounds{bounds_low_, bounds_high_},
step_size_, max_iters_,
goal_bias_, neighbor_radius_,
this->get_logger());

if (!planner->plan())
{
RCLCPP_WARN(this->get_logger(), "BP_RRTStar::plan() failed");
return false;
}

RCLCPP_INFO(this->get_logger(), "BP_RRTStar::plan() succeeded");

auto poses = planner->getPathMsg();
planner.reset();

out_path.header.stamp = this->get_clock()->now();
out_path.header.frame_id = "world";
out_path.poses = std::move(poses);

RCLCPP_INFO(this->get_logger(), "planTo: produced %zu waypoints", out_path.poses.size());
return true;
}


bool BPRRTPlannerNode::planCartesian(
  const std::vector<geometry_msgs::msg::Pose>& rrt_path,
  moveit_msgs::msg::RobotTrajectory& out_traj)
{
  RCLCPP_INFO(this->get_logger(), "planCartesian start (got %zu RRT waypoints)",
          rrt_path.size());

  // 1) Densify the RRT waypoints
  std::vector<geometry_msgs::msg::Pose> dense;
  const int DIVS = 5;
  // grab the current tool orientation once
  const auto ee_ori = move_group_.getCurrentPose().pose.orientation;
  for (size_t i = 1; i < rrt_path.size(); ++i)
  {
    const auto &A = rrt_path[i-1], &B = rrt_path[i];
   
    {
      geometry_msgs::msg::Pose p;
      p.position    = A.position;
      p.orientation = ee_ori;
      dense.push_back(p);
    }
    // intermediate points
    for (int k = 1; k < DIVS; ++k)
    {
      double t = double(k) / DIVS;
      geometry_msgs::msg::Pose p;
      p.position.x  = A.position.x + t * (B.position.x - A.position.x);
      p.position.y  = A.position.y + t * (B.position.y - A.position.y);
      p.position.z  = A.position.z + t * (B.position.z - A.position.z);
      p.orientation = ee_ori;
      dense.push_back(p);
    }
  }
  
  {
    geometry_msgs::msg::Pose p = rrt_path.back();
    p.orientation         = ee_ori;
    dense.push_back(p);
  }
  RCLCPP_INFO(this->get_logger(), "planCartesian: densified to %zu waypoints", dense.size());

  // 2) Compute Cartesian path (6-arg overload)
  moveit_msgs::msg::MoveItErrorCodes ec;
  moveit_msgs::msg::RobotTrajectory cart;
  double fraction = move_group_.computeCartesianPath(
      dense,               // waypoints
      0.005,               // eef_step
      0.1,                 // jump_threshold
      cart,                // output
      /*avoid_collisions=*/false,
      &ec                  // error code pointer
  );
  RCLCPP_INFO(this->get_logger(), "computeCartesianPath → frac=%.1f%%, err=%d",
          fraction * 100.0, ec.val);

  // 3) nothing was found
  if (ec.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS || fraction < 0.01)
  {
    RCLCPP_WARN(this->get_logger(), "Cartesian path only %.1f%% achievable—aborting",
            fraction * 100.0);
    return false;
  }

  // 4) Hand back the valid trajectory
  out_traj = std::move(cart);
  RCLCPP_INFO(this->get_logger(), "   → planCartesian succeeded: %zu trajectory points",
          out_traj.joint_trajectory.points.size());
  return true;
}

bool BPRRTPlannerNode::timeParameterizeTrajectory(moveit_msgs::msg::RobotTrajectory &traj_msg)
{
  auto kinematic_state = move_group_.getCurrentState();
  const auto* jmg = kinematic_state->getJointModelGroup(move_group_.getName());
  robot_trajectory::RobotTrajectory rt(kinematic_state->getRobotModel(), jmg->getName());
  rt.setRobotTrajectoryMsg(*kinematic_state, traj_msg);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  if (!iptp.computeTimeStamps(rt, 1.0, 1.0))
  {
    RCLCPP_WARN(this->get_logger(), "timeParameterizeTrajectory() failed");
    return false;
  }

  rt.getRobotTrajectoryMsg(traj_msg);
  return true;
}


bool BPRRTPlannerNode::executeTrajectory(const moveit_msgs::msg::RobotTrajectory &traj_msg)
{
  move_group_.setStartStateToCurrentState();
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::msg::RobotState start;
  moveit::core::robotStateToRobotStateMsg(*move_group_.getCurrentState(), start);
  plan.start_state_ = start;
  plan.trajectory_ = traj_msg;

  moveit_msgs::msg::DisplayTrajectory disp;
  disp.trajectory_start = start;
  disp.trajectory.push_back(traj_msg);
  cart_traj_pub_->publish(disp);

  auto res = move_group_.execute(plan);
  if (res != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(this->get_logger(), "executeTrajectory() failed: %d", res.val);
    return false;
  }
  return true;
}

bool BPRRTPlannerNode::computeJointTrajectory(
  const std::vector<geometry_msgs::msg::Pose>& waypoints,
  moveit_msgs::msg::RobotTrajectory& traj_msg)
{
  auto kinematic_state = move_group_.getCurrentState();
  const auto* jmg = kinematic_state->getJointModelGroup(move_group_.getName());

  robot_trajectory::RobotTrajectory rt(
    kinematic_state->getRobotModel(),
    jmg->getName());

  rt.addSuffixWayPoint(*kinematic_state, 0.0);

  std::vector<double> seed(jmg->getVariableCount());
  kinematic_state->copyJointGroupPositions(jmg, seed.data());

  const double TIMEOUT = 0.2;

  for (const auto& ee_pose : waypoints)
  {
    moveit::core::RobotState tmp(*kinematic_state);

    tmp.setJointGroupPositions(jmg, seed);

    if (!tmp.setFromIK(jmg, ee_pose, TIMEOUT))
    {
      RCLCPP_WARN(this->get_logger(), "IK failed for one RRT waypoint—skipping");
      continue;
    }

    tmp.copyJointGroupPositions(jmg, seed.data());

    rt.addSuffixWayPoint(tmp, 0.1);  // 0.1s between points for time-param
  }

  if (rt.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "computeJointTrajectory: no valid IK solutions");
    return false;
  }

  rt.getRobotTrajectoryMsg(traj_msg);
  return true;
}


}  // namespace bprrt

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bprrt::BPRRTPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}