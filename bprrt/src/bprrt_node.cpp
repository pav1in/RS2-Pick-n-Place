#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include "obstacle_manager.hpp"
#include "bprrt/bp_rrt_star_lib.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
using namespace std::chrono_literals;
#include <thread>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

// using bprrt::BP_RRTStar;
// using bprrt::Bounds;

class BpRrtNode 
  : public rclcpp::Node
  // , public std::enable_shared_from_this<BpRrtNode>
{
public:
  BpRrtNode(const rclcpp::NodeOptions & opts)
  : Node("bprrt_node", opts)
  {
    path_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "bprrt_path_markers", rclcpp::QoS(1).transient_local());
      ws_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "ws_markers", rclcpp::QoS(1).transient_local());
   

    // Declare parameters
    declare_parameter<std::vector<double>>("workspace_min", {-0.5, -0.5, 0.0});
    declare_parameter<std::vector<double>>("workspace_max", { 1,  1, 1.0});
    declare_parameter<int>("samples_stage1", 300);
    declare_parameter<int>("samples_stage2", 800);
    declare_parameter<int>("samples_stage3", 2000);
    declare_parameter<double>("goal_bias", 0.05);
    declare_parameter<double>("neighbor_radius", 0.1);
    declare_parameter<double>("step_size", 0.05);
    declare_parameter<std::vector<double>>("goal_pose_xyz", {0.3, 0.0, 0.6});
    declare_parameter<std::vector<double>>("goal_pose_rpy", {0.0, 0.0, 0.0});

    // Read parameters
    get_parameter("workspace_min", ws_min_);
    get_parameter("workspace_max", ws_max_);
    get_parameter("samples_stage1", s1_);
    get_parameter("samples_stage2", s2_);
    get_parameter("samples_stage3", s3_);
    get_parameter("goal_bias", gb_);
    get_parameter("neighbor_radius", nr_);
    get_parameter("step_size", ss_);
    std::vector<double> xyz, rpy;
    get_parameter("goal_pose_xyz", xyz);
    get_parameter("goal_pose_rpy", rpy);

    // Build goal pose
    // goal_pose_.position.x = xyz[0];
    // goal_pose_.position.y = xyz[1];
    // goal_pose_.position.z = xyz[2];
    // tf2::Quaternion q;
    // q.setRPY(rpy[0], rpy[1], rpy[2]);
    // goal_pose_.orientation = tf2::toMsg(q);
 

    // MoveIt setup
    // move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    //     rclcpp::Node::shared_from_this(), "manipulator");

    // Workspace partitioning
    
    // obstacle_mgr_.setWorkspaceCorners({ ws_min_, ws_max_ });
    // cubes_ = obstacle_mgr_.defineWorkspaceCubes();
    // visualizeWorkspaceCubes();
    // Hard-coded test obstacles: centers and 8-corner boxes
    obj_centres_ = {
      {0.2,  0.0, 0.5},
      {-0.1, 0.3, 0.7},
      {0.4, -0.2, 0.6}
    };
    obj_boxes_ = {
      // object 1 corners
      {{0.15,-0.05,0.45},{0.25,-0.05,0.45},{0.25,0.05,0.45},{0.15,0.05,0.45},
       {0.15,-0.05,0.55},{0.25,-0.05,0.55},{0.25,0.05,0.55},{0.15,0.05,0.55}},
      // object 2 corners
      {{-0.15,0.25,0.65},{-0.05,0.25,0.65},{-0.05,0.35,0.65},{-0.15,0.35,0.65},
       {-0.15,0.25,0.75},{-0.05,0.25,0.75},{-0.05,0.35,0.75},{-0.15,0.35,0.75}},
      // object 3 corners
      {{0.35,-0.25,0.55},{0.45,-0.25,0.55},{0.45,-0.15,0.55},{0.35,-0.15,0.55},
       {0.35,-0.25,0.65},{0.45,-0.25,0.65},{0.45,-0.15,0.65},{0.35,-0.15,0.65}}
    };

    // Timer to kick off planning
    plan_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&BpRrtNode::runBpRrt, this)
    );
    


  }
  void setMoveGroup(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg)
  {
    move_group_ = std::move(mg);
  }
  void setGoalPose(const geometry_msgs::msg::Pose &p) { goal_pose_ = p; }
  const geometry_msgs::msg::Pose& getGoalPose() const { return goal_pose_; }
  void setStartPose(const geometry_msgs::msg::Pose &p) { start_pose_ = p; }
  
  

private:
  void runBpRrt() {
    // if (!initialized_) {
    //     // Safely grab the BpRrtNode’s shared_ptr via its own enable_shared_from_this base:
    //     auto selfbp = static_cast<std::enable_shared_from_this<BpRrtNode>&>(*this)
    //                       .shared_from_this();
      
    //     // Now construct MoveGroupInterface with a valid shared_ptr<Node>
    //     move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    //         selfbp, "manipulator");
      
    //     initialized_ = true;  // only do this once
    //     RCLCPP_INFO(get_logger(), "MoveGroupInterface initialized");
    //   }
      
    // splitRunner -> P (handles volumes, weights internally)
    double margin = 0.1;  // configurable or parameter
    RCLCPP_INFO(get_logger(), "Using start pose: [%.3f, %.3f, %.3f]",
            start_pose_.position.x, start_pose_.position.y, start_pose_.position.z);
    RCLCPP_INFO(get_logger(), "Using goal pose:  [%.3f, %.3f, %.3f]",
                goal_pose_.position.x, goal_pose_.position.y, goal_pose_.position.z);
    std::vector<double> dynamic_min = {
      std::min(start_pose_.position.x, goal_pose_.position.x) - margin,
      std::min(start_pose_.position.y, goal_pose_.position.y) - margin,
      std::min(start_pose_.position.z, goal_pose_.position.z) - margin
    };

    std::vector<double> dynamic_max = {
      std::max(start_pose_.position.x, goal_pose_.position.x) + margin,
      std::max(start_pose_.position.y, goal_pose_.position.y) + margin,
      std::max(start_pose_.position.z, goal_pose_.position.z) + margin
    };


  obstacle_mgr_.setWorkspaceCorners({dynamic_min, dynamic_max});
  cubes_ = obstacle_mgr_.defineWorkspaceCubes();
  visualizeWorkspaceCubes();

  auto P = obstacle_mgr_.splitRunner(obj_centres_, obj_boxes_, cubes_);

  int start_idx = cellIndexOf(start_pose_);
  int goal_idx  = cellIndexOf(goal_pose_);

    // find start/goal indices
    // auto start = move_group_->getCurrentPose().pose;
    // int start_idx = cellIndexOf(start_pose_);
    // int goal_idx  = cellIndexOf(goal_pose_);

    // build P1, P2, P3
    std::vector<double> P1(27,0.0);
    P1[start_idx] = 1.0;

    auto nbrs = neighbourIndices(start_idx, goal_idx);
    double sum2 = 0;
    for(int i: nbrs) sum2 += P[i];
    std::vector<double> P2(27,0.0);
    for(int i: nbrs) P2[i] = P[i]/sum2;

    // configure planner
    Bounds full{toEigen(ws_min_), toEigen(ws_max_)};
    BP_RRTStar planner(full.low, full.high, full, ss_, s1_, gb_, nr_, get_logger());
    planner.setWorkspaceCubes(cubes_);

    // Stage 1
    planner.setMaxIters(s1_);
    planner.setSamplingWeights(P1);
    if(planner.plan()) { publishPath(planner.getPathMsg()); return; }

    // Stage 2
    planner.setMaxIters(s2_);
    planner.setSamplingWeights(P2);
    if(planner.plan()) { publishPath(planner.getPathMsg()); return; }

    // Stage 3
    planner.setMaxIters(s3_);
    planner.setSamplingWeights(P);
    if(planner.plan()) { publishPath(planner.getPathMsg()); return; }

    RCLCPP_WARN(get_logger(), "BP-RRT* failed all 3 stages");
  }
 void visualizePath(const std::vector<geometry_msgs::msg::Pose>& poses,
                   const std::vector<bool>& ik_success = {}) {
  visualization_msgs::msg::MarkerArray marker_array;

  // Line strip
  visualization_msgs::msg::Marker line_strip;
  line_strip.header.frame_id = "world";
  line_strip.header.stamp = now();
  line_strip.ns = "bprrt_path";
  line_strip.id = 0;
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::msg::Marker::ADD;
  line_strip.scale.x = 0.005;
  line_strip.color.r = 0.0f;
  line_strip.color.g = 1.0f;
  line_strip.color.b = 0.0f;
  line_strip.color.a = 1.0f;
  line_strip.lifetime = rclcpp::Duration(0, 0);

  line_strip.frame_locked = true;

  // Add each pose point to line strip and add a sphere marker
  for (size_t i = 0; i < poses.size(); ++i) {
    const auto& pose = poses[i];
    geometry_msgs::msg::Point pt;
    pt.x = pose.position.x;
    pt.y = pose.position.y;
    pt.z = pose.position.z;
    line_strip.points.push_back(pt);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = now();
    marker.ns = "bprrt_poses";
    marker.id = i + 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = 0.015;
    marker.scale.y = 0.015;
    marker.scale.z = 0.015;
    marker.lifetime = rclcpp::Duration(0, 0);

    marker.frame_locked = true;

    if (!ik_success.empty() && !ik_success[i]) {
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
    } else {
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
    }
    marker.color.b = 0.0f;
    marker.color.a = 0.9f;

    marker_array.markers.push_back(marker);
  }

  marker_array.markers.push_back(line_strip);
  path_marker_pub_->publish(marker_array);
  RCLCPP_INFO(this->get_logger(), "Published path and pose markers to RViz.");
}

void visualizeWorkspaceCubes() {
  visualization_msgs::msg::MarkerArray cube_markers;

  for (size_t i = 0; i < cubes_.size(); ++i) {
    const auto& cube = cubes_[i];
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "base_link";  // Make sure RViz uses this as fixed frame
    marker.header.stamp = now();
    marker.ns = "workspace";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = (cube.xmin() + cube.xmax()) / 2.0;
    marker.pose.position.y = (cube.ymin() + cube.ymax()) / 2.0;
    marker.pose.position.z = (cube.zmin() + cube.zmax()) / 2.0;
    marker.scale.x = cube.xmax() - cube.xmin();
    marker.scale.y = cube.ymax() - cube.ymin();
    marker.scale.z = cube.zmax() - cube.zmin();
    
    marker.color.r = 0.8f;
    marker.color.g = 0.1f;
    marker.color.b = 0.9f;
    marker.color.a = 0.4f;

    marker.lifetime = rclcpp::Duration(0, 0);  // persistent
    marker.frame_locked = true;

    cube_markers.markers.push_back(marker);
  }

  ws_marker_pub_->publish(cube_markers);
  RCLCPP_INFO(get_logger(), "Published %lu workspace cubes", cube_markers.markers.size());
}





  void publishPath(const std::vector<geometry_msgs::msg::Pose>& poses) {
  if (poses.empty()) {
    RCLCPP_ERROR(get_logger(), "No poses to plan to.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Attempting Cartesian path with %ld poses...", poses.size());
  moveit_msgs::msg::RobotTrajectory traj;
  double fraction = move_group_->computeCartesianPath(poses, 0.01, 0.0, traj);

  if (fraction >= 0.95 && !traj.joint_trajectory.points.empty()) {
    RCLCPP_INFO(get_logger(), "Cartesian path succeeded (%.1f%%).", fraction * 100.0);
    visualizePath(poses);  // All succeeded
    move_group_->execute(traj);
    return;
  }

  RCLCPP_WARN(get_logger(), "Cartesian path incomplete (%.1f%%). Falling back to IK.", fraction * 100.0);

  // Fallback: plan with IK
  auto robot_model = move_group_->getRobotModel();
  auto planning_group = move_group_->getName();
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(robot_model));

  kinematic_state->setToDefaultValues();

  robot_trajectory::RobotTrajectory rt(robot_model, planning_group);
  const auto* joint_model_group = robot_model->getJointModelGroup(planning_group);
  std::vector<bool> ik_success(poses.size(), false);

  for (size_t i = 0; i < poses.size(); ++i) {
    bool found_ik = kinematic_state->setFromIK(joint_model_group, poses[i], 0.2);
    if (!found_ik) {
      RCLCPP_WARN(get_logger(), "IK failed for pose %ld", i);
      continue;
    }
    rt.addSuffixWayPoint(kinematic_state, 0.2);
    ik_success[i] = true;
  }

  if (rt.empty()) {
    RCLCPP_ERROR(get_logger(), "All IK attempts failed.");
    visualizePath(poses, ik_success);  // show failures
    return;
  }

  // Time-parameterize
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  if (!iptp.computeTimeStamps(rt)) {
    RCLCPP_ERROR(get_logger(), "Time parameterization failed.");
    return;
  }

  moveit_msgs::msg::RobotTrajectory traj_msg;
  rt.getRobotTrajectoryMsg(traj_msg);

  visualizePath(poses, ik_success);
  move_group_->execute(traj_msg);
}


  int cellIndexOf(const geometry_msgs::msg::Pose& p) const {
    for(size_t i=0; i<cubes_.size(); ++i) {
      const auto& c = cubes_[i];
      double x=p.position.x, y=p.position.y, z=p.position.z;
      if(x>=c.xmin() && x<=c.xmax() &&
         y>=c.ymin() && y<=c.ymax() &&
         z>=c.zmin() && z<=c.zmax())
        return i;
    }
    RCLCPP_ERROR(get_logger(), "Pose outside workspace");
    return -1;
  }

  std::vector<int> neighbourIndices(int sidx, int gidx) const {
    auto idx3=[&](int i){int z=i/9, rem=i%9, y=rem/3, x=rem%3; return std::array<int,3>{x,y,z};};
    auto toFlat=[&](int x,int y,int z){return z*9+y*3+x;};
    auto s=idx3(sidx), g=idx3(gidx);
    int dx=(g[0]>s[0]?1:g[0]<s[0]?-1:0),
        dy=(g[1]>s[1]?1:g[1]<s[1]?-1:0),
        dz=(g[2]>s[2]?1:g[2]<s[2]?-1:0);
    std::vector<int> out;
    for(int ox=0;ox<=1;++ox) for(int oy=0;oy<=1;++oy) for(int oz=0;oz<=1;++oz){
      if(!ox&&!oy&&!oz) continue;
      int x=s[0]+ox*dx, y=s[1]+oy*dy, z=s[2]+oz*dz;
      if(x>=0&&x<3&&y>=0&&y<3&&z>=0&&z<3) out.push_back(toFlat(x,y,z));
    }
    return out;
  }

  Eigen::Vector3d toEigen(const std::vector<double>& v) const {
    return {v[0], v[1], v[2]};
  }
  
  

  // Members
  // bool initialized_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  
  ObstacleManager obstacle_mgr_;
  std::vector<CGAL::Iso_cuboid_3<CGAL::Epick>> cubes_;
  std::vector<std::vector<double>> obj_centres_;
  std::vector<std::vector<std::vector<double>>> obj_boxes_;
  geometry_msgs::msg::Pose goal_pose_;
  
  geometry_msgs::msg::Pose start_pose_;
  std::vector<double> ws_min_, ws_max_;

  int s1_, s2_, s3_;
  double gb_, nr_, ss_;
  rclcpp::TimerBase::SharedPtr plan_timer_;

  bool start= false;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ws_marker_pub_;


};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  opts.append_parameter_override("use_sim_time", true);

  auto node = std::make_shared<BpRrtNode>(opts);

  // Spin executor for state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // Wait for simulation time
  rclcpp::Clock sim_clock(RCL_ROS_TIME);
  while (rclcpp::ok() && sim_clock.now().nanoseconds() == 0) {
    RCLCPP_INFO(node->get_logger(), "Waiting for /clock...");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // MoveGroup setup
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");
  node->setMoveGroup(move_group);

  // Wait for valid current pose
  geometry_msgs::msg::PoseStamped ee_pose;
  rclcpp::Time start_time = node->now();
  rclcpp::Duration timeout(10, 0);
  bool received_pose = false;

  while (rclcpp::ok() && (node->now() - start_time) < timeout) {
    ee_pose = move_group->getCurrentPose();
    if (ee_pose.header.stamp.sec > 0 || ee_pose.header.stamp.nanosec > 0) {
      received_pose = true;
      break;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for joint state updates...");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  if (!received_pose) {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for valid robot state.");
    rclcpp::shutdown();
    return 1;
  }

  // Set poses in node
  auto start_pose = ee_pose.pose;
  geometry_msgs::msg::Pose goal = start_pose;
  goal.position.x += 0.05;
  goal.position.y += 0.05;
  goal.position.z -= 0.05;

  node->setStartPose(start_pose);
  node->setGoalPose(goal);

  RCLCPP_INFO(node->get_logger(), "Start: [%.3f, %.3f, %.3f]", 
              start_pose.position.x, start_pose.position.y, start_pose.position.z);
  RCLCPP_INFO(node->get_logger(), "Goal:  [%.3f, %.3f, %.3f]", 
              goal.position.x, goal.position.y, goal.position.z);

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}



// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);

//   // 1) build your node
//   rclcpp::NodeOptions opts;
//   opts.automatically_declare_parameters_from_overrides(true);
//   auto node = std::make_shared<BpRrtNode>(opts);

//   // 2) attach MoveGroupInterface
//   auto mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//     node, "ur_manipulator");
//   node->setMoveGroup(mg);

//   // 3) now that move_group_ is live, compute & bake in your offset-goal:
//   {
//     auto current = mg->getCurrentPose();        // PoseStamped
//     geometry_msgs::msg::Pose p = current.pose;  // copy
//     p.position.x += 0.20;                       // +20 cm X
//     p.position.z -= 0.20;                       // –20 cm Z
//     node->setGoalPose(p);
//   }

//   RCLCPP_INFO(node->get_logger(),
//               "Goal set to 20 cm right/down from current EE: [%.2f,%.2f,%.2f]",
//               node->getGoalPose().position.x,
//               node->getGoalPose().position.y,
//               node->getGoalPose().position.z);

//   rclcpp::spin(node);
//   rclcpp::shutdown();
// }
// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions opts;
//   opts.automatically_declare_parameters_from_overrides(true);
//   auto node = std::make_shared<BpRrtNode>(opts);

//   // Start executor for MoveGroup state monitor
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   std::thread spin_thread([&executor]() { executor.spin(); });

//   // Create MoveGroupInterface
//   auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//       node, "ur_manipulator");
//   node->setMoveGroup(move_group);

//   // Wait for a valid current pose
//   geometry_msgs::msg::PoseStamped ee_pose;
//   rclcpp::Rate rate(10); // 10 Hz
//   while (rclcpp::ok()) {
//     ee_pose = move_group->getCurrentPose();
//     if (ee_pose.header.stamp.sec != 0) break;  // Valid timestamp
//     RCLCPP_INFO(node->get_logger(), "Waiting for joint state updates...");
//     rate.sleep();
//   }

//   auto start_pose = ee_pose.pose;
//   RCLCPP_INFO(node->get_logger(),
//               "Start pose: [%.3f, %.3f, %.3f]",
//               start_pose.position.x,
//               start_pose.position.y,
//               start_pose.position.z);

//   // Set goal pose
//   geometry_msgs::msg::Pose goal = start_pose;
//   goal.position.x += 0.05;
//   goal.position.y += 0.05;
//   goal.position.z -= 0.20;
//   node->setGoalPose(goal);
//   RCLCPP_INFO(node->get_logger(),
//               "Goal set to: [%.3f, %.3f, %.3f]",
//               goal.position.x,
//               goal.position.y,
//               goal.position.z);

//   rclcpp::shutdown();
//   spin_thread.join();
//   return 0;
// }
// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);

//   rclcpp::NodeOptions opts;
//   opts.automatically_declare_parameters_from_overrides(true);
//   opts.append_parameter_override("use_sim_time", true);

//   auto node = std::make_shared<BpRrtNode>(opts);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   std::thread spin_thread([&executor]() { executor.spin(); });

//   // Wait for sim clock
//   auto sim_clock = rclcpp::Clock(RCL_ROS_TIME);
//   while (rclcpp::ok() && sim_clock.now().nanoseconds() == 0) {
//     RCLCPP_INFO(node->get_logger(), "Waiting for /clock to start publishing...");
//     std::this_thread::sleep_for(100ms);
//   }

//   // Initialize MoveGroup
//   auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//       node, "ur_manipulator");
//   node->setMoveGroup(move_group);

//   // Wait for robot state to be valid (timestamp > 0)
//   RCLCPP_INFO(node->get_logger(), "Waiting for current robot state from MoveIt...");
//   const rclcpp::Time start_time = sim_clock.now();
//   const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(10);
//   bool state_ok = false;

//   while (rclcpp::ok() && (sim_clock.now() - start_time) < timeout) {
//     moveit::core::RobotStatePtr state = move_group->getCurrentState(1.0);
//     if (state && state->getRobotModel()) {
//       state_ok = true;
//       break;
//     }
//     std::this_thread::sleep_for(100ms);
//   }

//   if (!state_ok) {
//     RCLCPP_ERROR(node->get_logger(), "Timed out waiting for MoveIt robot state.");
//     rclcpp::shutdown();
//     return 1;
//   }

//   // Continue with pose logic
//   geometry_msgs::msg::Pose start_pose = move_group->getCurrentPose().pose;
//   RCLCPP_INFO(node->get_logger(), "Start pose: [%.3f, %.3f, %.3f]",
//               start_pose.position.x, start_pose.position.y, start_pose.position.z);

//   geometry_msgs::msg::Pose goal = start_pose;
//   goal.position.x += 0.05;
//   goal.position.y += 0.05;
//   goal.position.z -= 0.20;
//   node->setGoalPose(goal);
//   RCLCPP_INFO(node->get_logger(), "Goal set to: [%.3f, %.3f, %.3f]",
//               goal.position.x, goal.position.y, goal.position.z);

//   spin_thread.join();
//   rclcpp::shutdown();
//   return 0;
// }
///////////////////////////////////////////////////////////////////////////////////////////////////
// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);

//   rclcpp::NodeOptions opts;
//   opts.automatically_declare_parameters_from_overrides(true);
//   opts.append_parameter_override("use_sim_time", true);

//   auto node = std::make_shared<BpRrtNode>(opts);

//   // Spin for state monitor and other callbacks
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   std::thread spin_thread([&executor]() { executor.spin(); });

//   // Wait for /clock
//   rclcpp::Clock sim_clock(RCL_ROS_TIME);
//   while (rclcpp::ok() && sim_clock.now().nanoseconds() == 0) {
//     RCLCPP_INFO(node->get_logger(), "Waiting for /clock to start publishing...");
//     std::this_thread::sleep_for(std::chrono::milliseconds(100));
//   }

//   // Initialize MoveGroup
//   auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");
//   node->setMoveGroup(move_group);

//   // Wait for a valid pose (timestamp > 0)
//   RCLCPP_INFO(node->get_logger(), "Waiting for current robot state...");
//   geometry_msgs::msg::PoseStamped ee_pose;
//   rclcpp::Time start_time = node->now();
//   rclcpp::Duration timeout(10, 0);  

//   bool received_pose = false;
//   while (rclcpp::ok() && (node->now() - start_time) < timeout) {
//     ee_pose = move_group->getCurrentPose();
//     if (ee_pose.header.stamp.sec > 0 || ee_pose.header.stamp.nanosec > 0) {
//       received_pose = true;
//       break;
//     }
//     RCLCPP_INFO(node->get_logger(), "Waiting for joint state updates...");
//     std::this_thread::sleep_for(std::chrono::milliseconds(200));
//   }

//   if (!received_pose) {
//     RCLCPP_ERROR(node->get_logger(), "Timed out waiting for valid robot state.");
//     rclcpp::shutdown();
//     return 1;
//   }

//   // Pose received — compute goal
//   geometry_msgs::msg::Pose start_pose = ee_pose.pose;
//   RCLCPP_INFO(node->get_logger(),
//               "Start pose: [%.3f, %.3f, %.3f]",
//               start_pose.position.x,
//               start_pose.position.y,
//               start_pose.position.z);

//   geometry_msgs::msg::Pose goal = start_pose;
//   goal.position.x += 0.05;
//   goal.position.y += 0.05;
//   goal.position.z -= 0.05;
//   node->setGoalPose(goal);

//   RCLCPP_INFO(node->get_logger(),
//               "Goal set to: [%.3f, %.3f, %.3f]",
//               goal.position.x,
//               goal.position.y,
//               goal.position.z);

//   spin_thread.join();
//   rclcpp::shutdown();
//   return 0;
// }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// #include <moveit/robot_state/robot_state.h>
// #include <moveit/robot_trajectory/robot_trajectory.h>

// void BpRrtNode::publishPath(const std::vector<geometry_msgs::msg::Pose>& poses) {
//   if (poses.empty()) {
//     RCLCPP_ERROR(get_logger(), "No poses to plan to.");
//     return;
//   }

//   auto robot_model = move_group_->getRobotModel();
//   auto planning_group = move_group_->getName();

//   robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));
//   kinematic_state->setToDefaultValues();

//   robot_trajectory::RobotTrajectory rt(robot_model, planning_group);

//   for (size_t i = 0; i < poses.size(); ++i) {
//     bool found_ik = kinematic_state->setFromIK(
//       robot_model->getJointModelGroup(planning_group),
//       poses[i],
//       0.2  // timeout per pose
//     );

//     if (!found_ik) {
//       RCLCPP_WARN(get_logger(), "IK failed for pose %ld. Skipping.", i);
//       continue;
//     }

//     rt.addSuffixWayPoint(kinematic_state, 0.2);  // 0.2s gap between poses
//   }

//   if (rt.getWayPointCount() == 0) {
//     RCLCPP_ERROR(get_logger(), "All IK attempts failed.");
//     return;
//   }

//   // Time-parametrize the trajectory
//   trajectory_processing::IterativeParabolicTimeParameterization iptp;
//   bool success = iptp.computeTimeStamps(rt);

//   if (!success) {
//     RCLCPP_ERROR(get_logger(), "Time parameterization failed.");
//     return;
//   }

//   moveit_msgs::msg::RobotTrajectory traj_msg;
//   rt.getRobotTrajectoryMsg(traj_msg);

//   // Visualize and execute
//   visualizePath(poses);
//   move_group_->execute(traj_msg);
// }