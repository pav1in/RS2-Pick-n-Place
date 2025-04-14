#include "ur3_object_picking/bp_rrt_planner.h"
#include "ur3_object_picking/box_tracker.h"
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>  
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

void addFloorCollisionObject()
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::CollisionObject floor;
    floor.header.frame_id = "world";
    floor.id = "floor";

    // Define a large flat box as the floor
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {2.0, 2.0, 0.01};  // Width, Depth, Height

    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = -0.005;  // Slightly below z=0

    floor.primitives.push_back(primitive);
    floor.primitive_poses.push_back(pose);
    floor.operation = floor.ADD;

    planning_scene_interface.applyCollisionObjects({floor});
    ROS_INFO("Added floor collision object to planning scene.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bp_rrt_planner_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
     // Optional: Unpause Gazebo physics
    if (ros::service::exists("/gazebo/unpause_physics", true)) {
        std_srvs::Empty srv;
        if (ros::service::call("/gazebo/unpause_physics", srv))
            ROS_INFO("Gazebo physics unpaused successfully.");
        else
            ROS_WARN("Failed to unpause Gazebo physics.");
    }
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPlanningTime(5.0);
    move_group.setPlannerId("RRTConnectkConfigDefault");

    addFloorCollisionObject();

    BpRrtPlanner planner(move_group);
    BoxTracker tracker(nh);

    ros::Duration(2.0).sleep();  // Give tracker time to gather box data
    auto boxes = tracker.getTrackedBoxes();

    if (boxes.empty()) {
        ROS_WARN("No tracked boxes found. Exiting.");
        return 1;
    }

    for (size_t i = 0; i < boxes.size(); ++i)
    {
        auto it = std::next(boxes.begin(), i);
        if (it == boxes.end()) break;
        geometry_msgs::Pose pickup_pose = it->second;
        pickup_pose.position.z += 0.1;  // Offset for safe approach

        moveit::planning_interface::MoveGroupInterface::Plan pickup_plan;
        if (!planner.planToGoal(pickup_pose, pickup_plan)) {
            ROS_WARN("Planning to pickup for box %lu failed. Skipping.", i);
            continue;
        }

        ROS_INFO("Executing pickup trajectory for box %lu", i);
        move_group.execute(pickup_plan);
        ros::Duration(1.0).sleep();

        // Lift vertically
        geometry_msgs::Pose lift_pose = pickup_pose;
        lift_pose.position.z += 0.3;

        moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
        if (!planner.planToGoal(lift_pose, lift_plan)) {
            ROS_WARN("Lift failed for box %lu. Skipping drop.", i);
            continue;
        }

        move_group.execute(lift_plan);
        ros::Duration(1.0).sleep();

        // Drop-off pose (static)
        geometry_msgs::Pose drop_pose = lift_pose;
        drop_pose.position.x = -0.5;
        drop_pose.position.y = 0.0;
        drop_pose.position.z = 0.31;  // Drop from same height

        moveit::planning_interface::MoveGroupInterface::Plan drop_plan;
        if (!planner.planToGoal(drop_pose, drop_plan)) {
            ROS_WARN("Drop-off failed for box %lu.", i);
            continue;
        }

        move_group.execute(drop_plan);
        ros::Duration(1.0).sleep();
    }

    ros::shutdown();
    return 0;
}

