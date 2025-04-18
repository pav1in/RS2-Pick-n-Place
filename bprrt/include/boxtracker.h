#pragma once

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <map>
#include <string>

class BoxTracker
{
public:
    BoxTracker(ros::NodeHandle& nh);
    const std::map<std::string, geometry_msgs::Pose>& getTrackedBoxes() const;

private:
    void modelCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    geometry_msgs::Pose transformToWorld(const geometry_msgs::Pose& pose, const std::string& target_frame);

    ros::Subscriber sub_;
    std::map<std::string, geometry_msgs::Pose> boxes_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
