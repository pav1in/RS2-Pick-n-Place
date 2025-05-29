// File: src/boxtracker.cpp

#include "bprrt/boxtracker.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/exceptions.h>
#include <regex>
#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

BoxTracker::BoxTracker()
: Node("box_tracker_node")
{
  // 1) TF buffer & listener
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 2) Publisher for the detected boxes (PoseArray)
  pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "/spawned_boxes", 10);

  // 3) Publisher for single box poses (PoseStamped)
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/box_pose", 10);

  // 4) Publisher for consolidated markers
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/tracked_boxes", 10);

  // 5) Subscribers for ROI and OBB markers
  roi_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
    "/roi_marker",  10,
    std::bind(&BoxTracker::roiCallback, this, std::placeholders::_1));

  obb_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
    "/obb_marker", 10,
    std::bind(&BoxTracker::obbCallback, this, std::placeholders::_1));

  // 6) Timer at 10 Hz for TF-based PoseArray
  timer_ = this->create_wall_timer(
    100ms, std::bind(&BoxTracker::timerCallback, this));
}

void BoxTracker::roiCallback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(marker_mutex_);
  latest_roi_ = *msg;
  publishMarkers();
}

void BoxTracker::obbCallback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(marker_mutex_);
  latest_obb_ = *msg;
  publishMarkers();
}

void BoxTracker::publishMarkers()
{
  visualization_msgs::msg::MarkerArray array;
  {
    std::lock_guard<std::mutex> lock(marker_mutex_);
    if (!latest_roi_.header.frame_id.empty()) {
      array.markers.push_back(latest_roi_);

      // Also republish ROI as a PoseStamped on /box_pose
      geometry_msgs::msg::PoseStamped ps;
      ps.header = latest_roi_.header;
      ps.pose   = latest_roi_.pose;
      pose_pub_->publish(ps);
    }
    if (!latest_obb_.header.frame_id.empty()) {
      array.markers.push_back(latest_obb_);
    }
  }
  if (!array.markers.empty()) {
    marker_pub_->publish(array);
  }
}

void BoxTracker::timerCallback()
{
  // Prepare an empty PoseArray
  geometry_msgs::msg::PoseArray arr;
  arr.header.stamp    = this->get_clock()->now();
  arr.header.frame_id = "world";

  // Pick up all frame names and look for box frames
  const auto frames = tf_buffer_->getAllFrameNames();
  std::regex box_re{R"(^(.+)/link$)"};

  for (const auto & frame : frames) {
    std::smatch m;
    if (!std::regex_match(frame, m, box_re)) {
      continue;
    }

    // Lookup the transform "world" <- frame
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(
            "world",      // target frame
            frame,         // source frame
            tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(),
        "TF lookup failed for %s: %s", frame.c_str(), ex.what());
      continue;
    }

    // Unpack into a Pose
    geometry_msgs::msg::Pose p;
    p.position.x  = t.transform.translation.x;
    p.position.y  = t.transform.translation.y;
    p.position.z  = t.transform.translation.z;
    p.orientation = t.transform.rotation;
    arr.poses.push_back(p);
  }

  // Only publish if we found at least one box
  if (!arr.poses.empty()) {
    pub_->publish(arr);
  }
}
