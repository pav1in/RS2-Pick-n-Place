#include "bprrt_node.h"
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/robot_state/conversions.h>  
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  


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

  nh_private.param("plan_rate", plan_rate_, 1.0);
  ROS_INFO("[bprrt_node] max_iters = %d", max_iters_);
  // --- subscriber to buffer the box messages ---
  boxes_sub_ = nh_global.subscribe(
    "/spawned_boxes", 10,
    &BPRRTPlannerNode::onBoxMessage, this);

  // --- publishers for RRT path & MoveIt trajectory ---
  path_pub_ = nh_global.advertise<geometry_msgs::PoseArray>(
                "/bprrt_path", 1, /*latch=*/true);
  cart_traj_pub_ = nh_global.advertise<moveit_msgs::DisplayTrajectory>(
                     "/move_group/display_planned_path", 1, /*latch=*/true);

  // --- timer to trigger planning at fixed rate ---
  plan_timer_ = nh_global.createTimer(
                   ros::Duration(1.0/plan_rate_),
                   &BPRRTPlannerNode::onPlanTimer,
                   this);

  latest_boxes_stamp_ = ros::Time(0);
  ROS_INFO("[bprrt_node] initialized (plan_rate=%.2f Hz)", plan_rate_);
}

void BPRRTPlannerNode::onBoxMessage(const geometry_msgs::PoseArray::ConstPtr& msg)  {
  // 1) if count changed → new data
  if (msg->poses.size() != buf_poses_.size())
  {
    buf_poses_   = msg->poses;
    new_data_    = true;
    return;
  }

  // 2) else compare each pose
  for (size_t i = 0; i < msg->poses.size(); ++i)
  {
    const auto &A = buf_poses_[i].position;
    const auto &B = msg->poses[i].position;
    double dx = A.x - B.x, dy = A.y - B.y, dz = A.z - B.z;
    if (std::sqrt(dx*dx + dy*dy + dz*dz) > replan_dist_thresh_)
    {
      buf_poses_ = msg->poses;
      new_data_  = true;
      return;
    }
  }

}

void BPRRTPlannerNode::onPlanTimer(const ros::TimerEvent&)
{
 
  if (!new_data_) 
    return;
  new_data_ = false;

  ROS_INFO("[bprrt_node] onPlanTimer: detected real box-change → planning");

  // 1) read current end-effector position
  auto ee = move_group_.getCurrentPose();
  Eigen::Vector3d start_pt{
    ee.pose.position.x,
    ee.pose.position.y,
    ee.pose.position.z
  };

  // 2) pick the first box and set goal 10cm above it
  if (buf_poses_.empty())
  {
    ROS_WARN("[bprrt_node] no boxes in buffer!");
    return;
  }
  const double tool_offset = 0.10;  
  double box_z = buf_poses_[0].position.z;
  double min_z  = box_z + tool_offset;

  Eigen::Vector3d goal_pt{
    buf_poses_[0].position.x,
    buf_poses_[0].position.y,
    min_z
  };

  // 3) run RRT*
  geometry_msgs::PoseArray rrt_path;
  if (!planTo(start_pt, goal_pt, rrt_path))
  {
    ROS_WARN("[bprrt_node] planTo() failed");
    return;
  }

  // 4) publish raw RRT waypoints for visualization
  ROS_INFO("[bprrt_node] publishing %zu RRT waypoints",
           rrt_path.poses.size());
  path_pub_.publish(rrt_path);

  // --- now force “face down” + floor clamp ---

  // roll +90° about tool-X so tool-Y→world-Z↓
  tf2::Quaternion down_q;
  down_q.setRPY(M_PI_2, 0.0, 0.0);
  down_q.normalize();
  geometry_msgs::Quaternion down_ori = tf2::toMsg(down_q);

  for (auto &wp : rrt_path.poses)
  {
    // clamp above the “floor” (box top + offset)
    wp.position.z  = std::max(wp.position.z, min_z);
    // force the face-Y of the flange to point down
    wp.orientation = down_ori;
  }

  // --- joint-space execution pipeline ---

  // 5) collect adjusted waypoints
  std::vector<geometry_msgs::Pose> waypoints(
    rrt_path.poses.begin(),
    rrt_path.poses.end());

  // 6) IK → joint trajectory
  moveit_msgs::RobotTrajectory joint_traj;
  ROS_INFO("[bprrt_node] computing joint-space trajectory");
  if (!computeJointTrajectory(waypoints, joint_traj))
  {
    ROS_WARN("[bprrt_node] computeJointTrajectory() failed");
    return;
  }

  // 7) time-parameterize
  ROS_INFO("[bprrt_node] time-parameterizing trajectory");
  if (!timeParameterizeTrajectory(joint_traj))
  {
    ROS_WARN("[bprrt_node] timeParameterizeTrajectory() failed");
    return;
  }

  // 8) execute on the robot
  ROS_INFO("[bprrt_node] executing joint trajectory");
  executeTrajectory(joint_traj);
}

bool BPRRTPlannerNode::planTo(const Eigen::Vector3d& start_pt,
  const Eigen::Vector3d& goal_pt,
  geometry_msgs::PoseArray& out_path)
{
  ROS_INFO(" planTo: constructing BP_RRTStar(start=(%.2f,%.2f,%.2f), goal=(%.2f,%.2f,%.2f))",
  start_pt.x(), start_pt.y(), start_pt.z(),
  goal_pt.x(),  goal_pt.y(),  goal_pt.z());

  // 1) heap‐allocate the planner 
  auto planner = std::make_unique<BP_RRTStar>(
  start_pt, goal_pt,
  Bounds{bounds_low_, bounds_high_},
  step_size_, max_iters_,
  goal_bias_, neighbor_radius_);

  // 2) planning
  if (!planner->plan())
  {
  ROS_WARN("BP_RRTStar::plan() failed");
  return false;
  }
  ROS_INFO("BP_RRTStar::plan() succeeded");

  // 3) extract just the converged path
  auto poses = planner->getPathMsg();

  // 4) destroy the planner 
  planner.reset();

  // 5) publishable out_path
  out_path.header.stamp    = ros::Time::now();
  out_path.header.frame_id = "world";
  out_path.poses           = std::move(poses);

  ROS_INFO("planTo: produced %zu waypoints", out_path.poses.size());
  return true;
}

bool BPRRTPlannerNode::planCartesian(
  const std::vector<geometry_msgs::Pose>& rrt_path,
  moveit_msgs::RobotTrajectory& out_traj)
{
  ROS_INFO("planCartesian start (got %zu RRT waypoints)",
          rrt_path.size());

  // 1) Densify the RRT waypoints
  std::vector<geometry_msgs::Pose> dense;
  const int DIVS = 5;
  // grab the current tool orientation once
  const auto ee_ori = move_group_.getCurrentPose().pose.orientation;
  for (size_t i = 1; i < rrt_path.size(); ++i)
  {
    const auto &A = rrt_path[i-1], &B = rrt_path[i];
   
    {
      geometry_msgs::Pose p;
      p.position    = A.position;
      p.orientation = ee_ori;
      dense.push_back(p);
    }
    // intermediate points
    for (int k = 1; k < DIVS; ++k)
    {
      double t = double(k) / DIVS;
      geometry_msgs::Pose p;
      p.position.x  = A.position.x + t * (B.position.x - A.position.x);
      p.position.y  = A.position.y + t * (B.position.y - A.position.y);
      p.position.z  = A.position.z + t * (B.position.z - A.position.z);
      p.orientation = ee_ori;
      dense.push_back(p);
    }
  }
  
  {
    geometry_msgs::Pose p = rrt_path.back();
    p.orientation         = ee_ori;
    dense.push_back(p);
  }
  ROS_INFO("planCartesian: densified to %zu waypoints", dense.size());

  // 2) Compute Cartesian path (6-arg overload)
  moveit_msgs::MoveItErrorCodes ec;
  moveit_msgs::RobotTrajectory cart;
  double fraction = move_group_.computeCartesianPath(
      dense,               // waypoints
      0.005,               // eef_step
      0.1,                 // jump_threshold
      cart,                // output
      /*avoid_collisions=*/false,
      &ec                  // error code pointer
  );
  ROS_INFO("computeCartesianPath → frac=%.1f%%, err=%d",
          fraction * 100.0, ec.val);

  // 3) nothing was found
  if (ec.val != moveit_msgs::MoveItErrorCodes::SUCCESS || fraction < 0.01)
  {
    ROS_WARN("Cartesian path only %.1f%% achievable—aborting",
            fraction * 100.0);
    return false;
  }

  // 4) Hand back the valid trajectory
  out_traj = std::move(cart);
  ROS_INFO("   → planCartesian succeeded: %zu trajectory points",
          out_traj.joint_trajectory.points.size());
  return true;
}

bool BPRRTPlannerNode::timeParameterizeTrajectory(
  moveit_msgs::RobotTrajectory &traj_msg)
{
  // wrap
  auto kinematic_state = move_group_.getCurrentState();
  const auto* jmg = kinematic_state->getJointModelGroup(
                      move_group_.getName());
  robot_trajectory::RobotTrajectory rt(
    kinematic_state->getRobotModel(), jmg->getName());
  rt.setRobotTrajectoryMsg(*kinematic_state, traj_msg);

  // apply IP-TP
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  if (!iptp.computeTimeStamps(rt, /*vel_scale*/1.0, /*acc_scale*/1.0))
  {
    ROS_WARN("[bprrt_node] timeParameterizeTrajectory() failed");
    return false;
  }

  rt.getRobotTrajectoryMsg(traj_msg);
  return true;
}

bool BPRRTPlannerNode::executeTrajectory(
  const moveit_msgs::RobotTrajectory &traj_msg)
{
  // a) build a MoveGroup plan
  move_group_.setStartStateToCurrentState();
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::RobotState start;
  moveit::core::robotStateToRobotStateMsg(
    *move_group_.getCurrentState(), start);
  plan.start_state_ = start;
  plan.trajectory_  = traj_msg;

  // b) publish for RViz
  moveit_msgs::DisplayTrajectory disp;
  disp.trajectory_start = start;
  disp.trajectory.push_back(traj_msg);
  cart_traj_pub_.publish(disp);

  // c) execute
  auto res = move_group_.execute(plan);
  if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_WARN("[bprrt_node] executeTrajectory() failed: %d", res.val);
    return false;
  }
  return true;
}

bool BPRRTPlannerNode::computeJointTrajectory(
  const std::vector<geometry_msgs::Pose>& waypoints,
  moveit_msgs::RobotTrajectory& traj_msg)
{
// a) current state & joint‐model group
auto kinematic_state = move_group_.getCurrentState();
const auto* jmg = kinematic_state->getJointModelGroup(
  move_group_.getName());
robot_trajectory::RobotTrajectory rt(
  kinematic_state->getRobotModel(),
  jmg->getName());
rt.addSuffixWayPoint(*kinematic_state, /*dt=*/0.0);
// b) current joint positions to a seed vector
std::vector<double> seed(jmg->getVariableCount());
kinematic_state->copyJointGroupPositions(jmg, seed.data());

// c)  solve IK
const unsigned int ATTEMPTS = 20;
const double TIMEOUT = 0.2;  // seconds
for (const auto& ee_pose : waypoints)
{
  moveit::core::RobotState tmp(*kinematic_state);

  // seed the solver
  tmp.setJointGroupPositions(jmg, seed.data());

  // attempt IK
  if (!tmp.setFromIK(jmg, ee_pose, ATTEMPTS, TIMEOUT))
  {
    ROS_WARN("[bprrt_node] IK failed for one RRT waypoint—skipping");
    continue;  // skip this waypoint
  }

  // copy the successful solution back into seed[]
  tmp.copyJointGroupPositions(jmg, seed.data());

  // append to the joint‐space trajectory (dt=0.1s so time‐param works)
  rt.addSuffixWayPoint(tmp, /*dt=*/0.1);
}

// d) ensure we got something
if (rt.empty())
{
  ROS_ERROR("[bprrt_node] computeJointTrajectory: no valid IK solutions");
  return false;
}

// e) extract into the ROS message
rt.getRobotTrajectoryMsg(traj_msg);
return true;
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
