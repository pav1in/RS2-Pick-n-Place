#include "ur3_object_picking/spawn_manager.h"
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <ctime>
#include <ros/package.h>

SpawnManager::SpawnManager(ros::NodeHandle& nh, const std::string& model_name)
{
    spawn_client_ = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    delete_client_ = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

    std::string model_path = ros::package::getPath("ur3_object_picking") + "/models/" + model_name + "/model.sdf";
    std::ifstream ifs(model_path);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    model_xml_ = buffer.str();

    std::srand(std::time(nullptr));
}

geometry_msgs::Pose SpawnManager::generateRandomPose()
{
    geometry_msgs::Pose pose;
    pose.position.x = 0.2f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 0.3f)); // [0.2, 0.5]
    pose.position.y = -0.3f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 0.6f)); // [-0.3, 0.3]
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
}

bool SpawnManager::spawnBox(const std::string& instance_name, const geometry_msgs::Pose& pose)
{
    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = instance_name;
    srv.request.model_xml = model_xml_;
    srv.request.robot_namespace = "/";
    srv.request.initial_pose = pose;
    srv.request.reference_frame = "world";

    if (spawn_client_.call(srv) && srv.response.success)
    {
        ROS_INFO("Spawned: %s", instance_name.c_str());
        spawned_boxes_.emplace_back(instance_name, pose);
        return true;
    }

    ROS_ERROR("Failed to spawn: %s", instance_name.c_str());
    return false;
}

bool SpawnManager::deleteBox(const std::string& instance_name)
{
    gazebo_msgs::DeleteModel srv;
    srv.request.model_name = instance_name;
    return delete_client_.call(srv) && srv.response.success;
}

// void SpawnManager::spawnRandomBoxes(int count)
// {
//     for (int i = 0; i < count; ++i)
//     {
//         ros::Time::waitForValid();
//         std::string name = "box_" + std::to_string(ros::Time::now().toNSec());
        
//         ros::Duration(0.01).sleep(); 
//         geometry_msgs::Pose pose = generateRandomPose();
//         spawnBox(name, pose);
//     }
// }
void SpawnManager::spawnRandomBoxes(int count)
{
    int retries = 0;

    for (int i = 0; i < count; ++i)
    {
        ros::Time::waitForValid();
        geometry_msgs::Pose pose;
        bool valid = false;

        // Try up to 50 times to find a non-colliding pose
        for (int attempt = 0; attempt < 50; ++attempt)
        {
            pose = generateRandomPose();
            valid = true;

            for (const auto& box : spawned_boxes_)
            {
                if (!isFarEnough(pose, box.second))
                {
                    valid = false;
                    break;
                }
            }

            if (valid) break;
        }

        if (!valid)
        {
            ROS_WARN("Could not find non-overlapping pose for box %d", i);
            continue;
        }

        std::string name = "box_" + std::to_string(ros::Time::now().toNSec());
        ros::Duration(0.01).sleep();  // ensure unique timestamp
        spawnBox(name, pose);
    }
}

const std::vector<std::pair<std::string, geometry_msgs::Pose>>& SpawnManager::getSpawnedBoxes() const
{
    return spawned_boxes_;
}

bool SpawnManager::isFarEnough(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b, double min_dist) const
{
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    return distance >= min_dist;
}

// #include "ur3_object_picking/spawn_manager.h"
// #include <fstream>
// #include <sstream>
// #include <cstdlib>
// #include <ros/package.h>
// #include <cmath>

// SpawnManager::SpawnManager(ros::NodeHandle& nh, const std::string& model_path) {
//     spawn_client_ = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
//     delete_client_ = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

//     std::ifstream ifs(model_path);
//     std::stringstream buffer;
//     buffer << ifs.rdbuf();
//     modelXML_ = buffer.str();
// }

// bool SpawnManager::spawnBox(const std::string& model_name, const geometry_msgs::Pose& pose) {
//     gazebo_msgs::SpawnModel srv;
//     srv.request.model_name = model_name;
//     srv.request.model_xml = modelXML_;
//     srv.request.robot_namespace = model_name;
//     srv.request.initial_pose = pose;
//     srv.request.reference_frame = "world";

//     if (spawn_client_.call(srv)) {
//         ROS_INFO_STREAM("Spawned: " << model_name);
//         return true;
//     } else {
//         ROS_ERROR_STREAM("Failed to spawn: " << model_name);
//         return false;
//     }
// }

// bool SpawnManager::deleteBox(const std::string& model_name) {
//     gazebo_msgs::DeleteModel srv;
//     srv.request.model_name = model_name;
//     return delete_client_.call(srv);
// }

// geometry_msgs::Pose SpawnManager::randomPose(float radius) {
//     geometry_msgs::Pose pose;
//     float angle = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI;
//     float r = static_cast<float>(rand()) / RAND_MAX * radius;

//     pose.position.x = r * cos(angle);
//     pose.position.y = r * sin(angle);
//     pose.position.z = 0.05;
//     pose.orientation.w = 1.0;
//     return pose;
// }
