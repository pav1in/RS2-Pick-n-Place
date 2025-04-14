#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <unordered_map>
#include <queue>
#include "ur3_object_picking/spawn_manager.h"

struct Task {
    geometry_msgs::Pose pickup;
    geometry_msgs::Pose drop;
    std::string name;
};

double euclideanDistance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    double dz = a.position.z - b.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool moveToPose(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Pose& pose, double wait = 1.0)
{
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        move_group.execute(plan);
        ros::Duration(wait).sleep();
        return true;
    }
    return false;
}

bool moveToHomePose(moveit::planning_interface::MoveGroupInterface& move_group, double wait_time = 1.0)
{
    std::vector<double> home_joint_values = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
    ros::Duration(0.5).sleep();
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

std::vector<int> dijkstra(int start, const std::vector<std::vector<double>>& cost)
{
    int n = cost.size();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> prev(n, -1);
    std::vector<bool> visited(n, false);
    dist[start] = 0.0;

    for (int i = 0; i < n; ++i)
    {
        int u = -1;
        for (int j = 0; j < n; ++j)
            if (!visited[j] && (u == -1 || dist[j] < dist[u])) u = j;

        if (dist[u] == std::numeric_limits<double>::infinity()) break;
        visited[u] = true;

        for (int v = 0; v < n; ++v)
            if (dist[u] + cost[u][v] < dist[v])
            {
                dist[v] = dist[u] + cost[u][v];
                prev[v] = u;
            }
    }

    return prev;  // Used to reconstruct path
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur3_task_planner_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(5.0);

    if (!moveToHomePose(move_group)) return 1;

    // Spawn 5 boxes
    SpawnManager manager(nh, "simple_box");
    ros::Duration(1.0).sleep();
    manager.spawnRandomBoxes(5);
    auto boxes = manager.getSpawnedBoxes();

    std::vector<Task> tasks;
    for (const auto& [name, pose] : boxes)
    {
        Task t;
        t.pickup = pose;
        t.pickup.position.z += 0.1;
        t.drop = t.pickup;
        t.drop.position.x = -0.5;
        t.drop.position.y = 0.0;
        t.drop.position.z += 0.2;
        t.name = name;
        tasks.push_back(t);
    }

    // Build distance matrix between all pickup/drop pairs
    int n = tasks.size();
    std::vector<geometry_msgs::Pose> points = {move_group.getCurrentPose().pose};
    for (const auto& t : tasks) points.push_back(t.pickup);

    std::vector<std::vector<double>> cost(points.size(), std::vector<double>(points.size(), 0.0));
    for (size_t i = 0; i < points.size(); ++i)
        for (size_t j = 0; j < points.size(); ++j)
            cost[i][j] = euclideanDistance(points[i], points[j]);

    // Get order using Dijkstra (could extend to TSP logic later)
    auto prev = dijkstra(0, cost);

    // For simplicity, execute in order of box index
    for (int i = 0; i < tasks.size(); ++i)
    {
        ROS_INFO_STREAM("Moving to pickup for: " << tasks[i].name);
        moveToPose(move_group, tasks[i].pickup);

        geometry_msgs::Pose lift = tasks[i].pickup;
        lift.position.z += 0.3;
        moveToPose(move_group, lift);

        ROS_INFO("Moving to drop-off...");
        moveToPose(move_group, tasks[i].drop);
    }

    moveToHomePose(move_group);
    ros::shutdown();
    return 0;
}