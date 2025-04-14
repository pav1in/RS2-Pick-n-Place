#include "ur3_object_picking/bp_rrt_planner.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>


BpRrtPlanner::BpRrtPlanner(moveit::planning_interface::MoveGroupInterface& move_group)
    : move_group_(move_group)
    
{
    // Optional: set additional planning parameters here
    move_group_.setPlannerId("RRTConnectkConfigDefault");
    move_group_.setPlanningTime(5.0);
    move_group_.setGoalTolerance(0.01);
}

bool BpRrtPlanner::executePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    move_group_.setStartStateToCurrentState();
    return (move_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

void BpRrtPlanner::moveToPose(const geometry_msgs::Pose& pose)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (planToGoal(pose, plan))
    {
        ROS_INFO("[BpRrtPlanner] Executing plan to goal pose");
        executePlan(plan);
    }
    else
    {
        ROS_WARN("[BpRrtPlanner] Planning failed for goal pose");
    }
}

bool BpRrtPlanner::planToGoal(const geometry_msgs::Pose& goal,
    moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    move_group_.setStartStateToCurrentState();
    move_group_.setPoseTarget(goal);

    return (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}