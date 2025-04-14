#include "ur3_object_picking/box_tracker.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

BoxTracker::BoxTracker(ros::NodeHandle& nh)
    : tf_listener_(tf_buffer_)
{
    sub_ = nh.subscribe("/gazebo/model_states", 10, &BoxTracker::modelCallback, this);
}

void BoxTracker::modelCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    boxes_.clear();
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i].find("box_") != std::string::npos)
        {
            geometry_msgs::Pose world_pose = transformToWorld(msg->pose[i], "world");
            boxes_[msg->name[i]] = world_pose;
        }
    }
}

geometry_msgs::Pose BoxTracker::transformToWorld(const geometry_msgs::Pose& pose, const std::string& target_frame)
{
    geometry_msgs::PoseStamped in, out;
    in.header.frame_id = "base_link";
    in.header.stamp = ros::Time(0);
    in.pose = pose;

    try
    {
        out = tf_buffer_.transform(in, target_frame, ros::Duration(0.5));
        return out.pose;
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Transform failed in BoxTracker: %s", ex.what());
        return pose;  // fallback
    }
}


const std::map<std::string, geometry_msgs::Pose>& BoxTracker::getTrackedBoxes() const
{
    return boxes_;
}
