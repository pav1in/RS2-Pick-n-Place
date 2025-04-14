#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/PoseArray.h>
#include "rrt_star_planner_lib.h"  // Our new library header

namespace rrt_star_planner_plugin {

class RRTStarPlanningContext : public planning_interface::PlanningContext {
public:
  RRTStarPlanningContext(const std::string &name, const std::string &group)
    : planning_interface::PlanningContext(name, group)
  {
    nh_ = ros::NodeHandle("~");
    path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("rrt_star_path", 1);

    // For testing, we load start and goal values from the parameter server.
    // In a complete system, the start should come from the current end-effector state,
    // and the goal from vision/object detection data.
    nh_.param("start_x", start_x_, 0.0);
    nh_.param("start_y", start_y_, 0.0);
    nh_.param("start_z", start_z_, 0.0);
    nh_.param("goal_x",  goal_x_, 5.0);
    nh_.param("goal_y",  goal_y_, 5.0);
    nh_.param("goal_z",  goal_z_, 5.0);
    nh_.param("x_min",   x_min_, -10.0);
    nh_.param("x_max",   x_max_, 10.0);
    nh_.param("y_min",   y_min_, -10.0);
    nh_.param("y_max",   y_max_, 10.0);
    nh_.param("z_min",   z_min_, -10.0);
    nh_.param("z_max",   z_max_, 10.0);
    nh_.param("step_size", step_size_, 0.5);
    nh_.param("max_iter", max_iter_, 500);
    nh_.param("goal_sample_rate", goal_sample_rate_, 0.1);
    nh_.param("neighbor_radius", neighbor_radius_, 1.0);

    // Create our RRT* planner library instance.
    planner_lib_ = new RRTStarPlannerLib(start_x_, start_y_, start_z_,
                                          goal_x_, goal_y_, goal_z_,
                                          x_min_, x_max_, y_min_, y_max_, z_min_, z_max_,
                                          step_size_, max_iter_, goal_sample_rate_, neighbor_radius_);
  }

  ~RRTStarPlanningContext() {
    delete planner_lib_;
  }

  virtual bool solve(planning_interface::MotionPlanResponse &res) override {
    bool success = planner_lib_->plan();
    std::vector<geometry_msgs::Pose> path = planner_lib_->getPath();
    publishPath(path);

    // Use the new error_code_ member as the old error_code is deprecated.
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    // No trajectory is generated at this stage.
    return success;
  }

  virtual bool solve(planning_interface::MotionPlanDetailedResponse &res) override {
    planning_interface::MotionPlanResponse resp;
    bool success = solve(resp);
    // Instead of assigning a single trajectory pointer to a vector,
    // we clear the detailed trajectory vector and leave it empty.
    res.trajectory_.clear();
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return success;
  }

  virtual void clear() override { }
  
  // We remove the override keyword if the signature doesn't match exactly.
  virtual void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &config) { }

  // Implement the required pure virtual function terminate().
  virtual bool terminate() override {
    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher path_pub_;
  RRTStarPlannerLib* planner_lib_;

  // Parameters (copied from parameter server)
  double start_x_, start_y_, start_z_;
  double goal_x_, goal_y_, goal_z_;
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
  double step_size_, goal_sample_rate_, neighbor_radius_;
  int max_iter_;

  void publishPath(const std::vector<geometry_msgs::Pose>& path) {
    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = "world";
    for (size_t i = 0; i < path.size(); i++) {
      pose_array.poses.push_back(path[i]);
    }
    path_pub_.publish(pose_array);
    ROS_INFO("Published path with %ld nodes.", path.size());
  }
};

class RRTStarPlannerManager : public planning_interface::PlannerManager {
public:
  RRTStarPlannerManager() { }
  
  virtual bool initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns) override {
    robot_model_ = model;
    return true;
  }

  virtual std::string getDescription() const override {
    return "RRTStarPlanner Plugin (using library)";
  }

  // Use the new MoveIt interface signature for getPlanningContext:
  virtual planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr &planning_scene,
      const planning_interface::MotionPlanRequest &req,
      moveit_msgs::MoveItErrorCodes &error_code) const override {
    planning_interface::PlanningContextPtr context(new RRTStarPlanningContext("RRTStarContext", req.group_name));
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return context;
  }

  virtual bool canServiceRequest(const planning_interface::MotionPlanRequest &req) const override {
    return true;
  }

private:
  robot_model::RobotModelConstPtr robot_model_;
};

} // namespace rrt_star_planner_plugin

PLUGINLIB_EXPORT_CLASS(rrt_star_planner_plugin::RRTStarPlannerManager, planning_interface::PlannerManager)
