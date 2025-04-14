#pragma once

#include <geometry_msgs/Pose.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

class BpRrtPlanner
{
public:
    explicit BpRrtPlanner(moveit::planning_interface::MoveGroupInterface& move_group);

    bool planToGoal(const geometry_msgs::Pose& goal,
                    moveit::planning_interface::MoveGroupInterface::Plan& plan);
    bool executePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
    void moveToPose(const geometry_msgs::Pose& pose);

private:
    moveit::planning_interface::MoveGroupInterface& move_group_;
};
