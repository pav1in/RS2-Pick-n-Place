#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include "ur3_object_picking/spawn_manager.h"
#include <std_srvs/Empty.h>

// Helper to move to a known home configuration
bool moveToHomePose(moveit::planning_interface::MoveGroupInterface& move_group, double wait_time = 1.0)
{
    std::vector<double> home_joint_values = {
        0.0,       // shoulder_pan_joint
        -1.57,     // shoulder_lift_joint
        1.57,      // elbow_joint
        -1.57,     // wrist_1_joint
        -1.57,     // wrist_2_joint
        0.0        // wrist_3_joint
    };

    ros::Duration(0.5).sleep();  // Let robot state update
    move_group.setStartStateToCurrentState();
    move_group.setJointValueTarget(home_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    if (move_group.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Moving to home pose...");
        move_group.move();
        ros::Duration(wait_time).sleep();
        return true;
    }

    ROS_WARN("Failed to plan to home pose.");
    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur3_planner_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    if (ros::service::exists("/gazebo/unpause_physics", true))
    {
        std_srvs::Empty srv;
        if (ros::service::call("/gazebo/unpause_physics", srv))
        {
            ROS_INFO("Gazebo physics unpaused successfully.");
        }
        else
        {
            ROS_WARN("Failed to unpause Gazebo physics.");
        }
    }
    else
    {
        ROS_WARN("/gazebo/unpause_physics service not available.");
    }
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(5.0);
    move_group.setGoalTolerance(0.01);  // Default is fine for real robot prep

    if (!moveToHomePose(move_group))
    {
        ROS_ERROR("Unable to proceed: Home pose movement failed.");
        return 1;
    }

    // Spawn boxes
    SpawnManager manager(nh, "simple_box");
    ros::Duration(1.0).sleep();  // Let Gazebo services become available
    manager.spawnRandomBoxes(3);
    auto boxes = manager.getSpawnedBoxes();
    if (boxes.empty())
    {
        ROS_WARN("No boxes available from SpawnManager.");
        return 1;
    }

    // Step 1: Move to pickup
    geometry_msgs::Pose target_pose = boxes[0].second;
    target_pose.position.z += 0.1;

    ros::Duration(0.5).sleep();  // Sync state monitor
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        ROS_INFO_STREAM("Moving to pickup at: x=" << target_pose.position.x
                                                  << ", y=" << target_pose.position.y
                                                  << ", z=" << target_pose.position.z);
        move_group.move();
        ros::Duration(1.0).sleep();  // Let robot settle

    // Step 2: Lift object vertically before moving to drop-off
    geometry_msgs::Pose lift_pose = target_pose;  // Copy from pickup pose
    lift_pose.position.z += 0.3;  // Lift 30cm upward

    ros::Duration(0.5).sleep();
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(lift_pose);

    moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
    success = (move_group.plan(lift_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        ROS_INFO("Lifting object...");
        move_group.execute(lift_plan);
        ros::Duration(1.0).sleep();

        // Step 3: Move to drop-off
        geometry_msgs::Pose drop_pose;
        drop_pose.orientation.w = 1.0;
        drop_pose.position.x = -0.5;
        drop_pose.position.y = 0.0;
        drop_pose.position.z = 0.31;  // Drop from same height (30cm)

        move_group.setStartStateToCurrentState();
        move_group.setPoseTarget(drop_pose);

        moveit::planning_interface::MoveGroupInterface::Plan drop_plan;
        success = (move_group.plan(drop_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            ROS_INFO("Moving to drop-off pose...");
            move_group.execute(drop_plan);
            ros::Duration(1.0).sleep();  // Simulate drop
        }
        else
        {
            ROS_WARN("Planning to drop-off pose failed.");
        }
    }
    else
    {
        ROS_WARN("Lifting object failed.");
    }

    ros::shutdown();
    return 0;
}
}



// #include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/Pose.h>
// #include "ur3_object_picking/spawn_manager.h"

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "ur3_planner_node");
//     ros::NodeHandle nh;
//     ros::AsyncSpinner spinner(1);
//     spinner.start();

//     moveit::planning_interface::MoveGroupInterface move_group("manipulator");
//     move_group.setPlannerId("RRTConnectkConfigDefault");
//     move_group.setPlanningTime(5.0);

//     SpawnManager manager(nh, "simple_box");
//     ros::Duration(1.0).sleep();  // Let Gazebo services become available

//     manager.spawnRandomBoxes(3);
//     auto boxes = manager.getSpawnedBoxes();
//     if (boxes.empty())
//     {
//         ROS_WARN("No boxes available from SpawnManager.");
//         return 1;
//     }

//     // ---- Step 1: Plan to pickup location ----
//     geometry_msgs::Pose target_pose = boxes[0].second;
//     target_pose.position.z += 0.05;  // Approach above box

//     // move_group.setStartStateToCurrentState();
//     // moveit::core::RobotStatePtr current_state = move_group.getCurrentState(2.0);
//     // move_group.setStartState(*current_state);
//     // ros::Duration(0.5).sleep();  // Let controller catch up
//     // move_group.setPoseTarget(target_pose);
//     move_group.setStartStateToCurrentState();
//     move_group.setPoseTarget(target_pose);

//     // Force update from current state
//     moveit::core::RobotStatePtr current_state = move_group.getCurrentState(2.0);
//     move_group.setStartState(*current_state);
//     ros::Duration(0.5).sleep();
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//     if (success && !plan.trajectory_.joint_trajectory.points.empty())
//     {
//         ROS_INFO_STREAM("Moving to pickup at: x=" << target_pose.position.x
//                                                   << ", y=" << target_pose.position.y
//                                                   << ", z=" << target_pose.position.z);
//         ros::Duration(1.0).sleep();  // Let robot settle before executing
//         move_group.execute(plan);
//         ros::Duration(2.0).sleep(); // Simulate pickup

//         // ---- Step 2: Plan to drop-off location ----
//         geometry_msgs::Pose drop_pose = target_pose;
//         drop_pose.position.x = 0.3;
//         drop_pose.position.y = -0.4;

//         move_group.setStartStateToCurrentState();
//         move_group.setPoseTarget(drop_pose);

//         success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//         if (success && !plan.trajectory_.joint_trajectory.points.empty())
//         {
//             ROS_INFO("Moving to drop-off pose...");
//             ros::Duration(1.0).sleep();  // Let robot settle before executing
//             move_group.execute(plan);
//             ros::Duration(1.0).sleep(); // Simulate drop
//         }
//         else
//         {
//             ROS_WARN("Planning to drop-off pose failed.");
//         }
//     }
//     else
//     {
//         ROS_WARN("Planning to pickup pose failed.");
//     }

//     ros::shutdown();
//     return 0;
// }
