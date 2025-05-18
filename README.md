# Perception Subsystem – ROS 2 (YOLOv8 + RealSense Integration)

This branch provides a real-time perception pipeline for robotic pick-and-place, combining an Intel RealSense RGB-D camera with YOLOv8 object detection in ROS 2.  
It publishes 2D and 3D object localization, segmented point clouds, and live visualization topics for integration with downstream planning and manipulation subsystems.

---

## Table of Contents

- [Purpose](#purpose)
- [Key Topics](#key-topics)
- [Installation and Setup](#installation-and-setup)
- [Subsystem Demonstration (Real Hardware)](#subsystem-demonstration-real-hardware)
- [RViz Visualization Topics](#rviz-visualization-topics)
- [Troubleshooting](#troubleshooting)

---

## Purpose

Detect and localize objects in real time using YOLOv8 and an Intel RealSense camera.  
Outputs 3D poses and segmented point clouds for downstream manipulation and planning.

---

## Key Topics

| Type        | Topic                                | Description                                      |
|-------------|--------------------------------------|--------------------------------------------------|
| Subscribes  | `/camera/color/image_raw`            | RGB image stream (input to YOLOv8)               |
| Subscribes  | `/camera/aligned_depth_to_color/image_raw` | Aligned depth image                        |
| Subscribes  | `/camera/depth/color/points`         | Organized point cloud                            |
| Subscribes  | `/camera/color/camera_info`          | Camera intrinsics                                |
| Publishes   | `/detected_object_pose`              | 3D pose of detected object(s)                    |
| Publishes   | `/roi_marker`                        | Sphere marker for object centroid                |
| Publishes   | `/obb_marker`                        | Cube marker for bounding box (3D)                |
| Publishes   | `/segmented_roi`                     | Segmented point cloud for detected object        |
| Publishes   | `/yolo/image`                        | Annotated image with detection overlays          |

---

## Installation and Setup

### 1. Switch to the Perception Branch
```bash
cd ~/git/RS2-Pick-n-Place
git checkout Perception
```

### 2. Symlink only the yolov8_object_detector package into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
ln -s ~/git/RS2-Pick-n-Place/yolov8_object_detector yolov8_object_detector
```
If you switch branches, remove the symlink before linking a different version.

### 3. Install Required Python Packages
```bash
pip3 install --user ultralytics onnx onnxruntime ros2_numpy opencv-python
```
### 4. Install Required ROS 2 Packages
```bash
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-tf-transformations
sudo apt install ros-humble-librealsense2* ros-humble-realsense2-camera
```

### 5. Install and Configure the RealSense ROS 2 Wrapper
```bash
sudo usermod -a -G dialout $USER
wget -O ~/99-realsense.rules https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
sudo mv ~/99-realsense.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```
Note: You may need to log out and back in for group permissions to take effect.

---
## Subsystem Demonstration in Real Life

### 1.	In Terminal A, launch the real sense node
```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  pointcloud.enable:=true \
  depth_module.profile:=640x480x30
```

### 2.	In Terminal B, Run the YOLOv8 Object Detector Node
```bash
cd ~/ros2_ws
colcon build --symlink-install yolov8_object_detector
source install/setup.bash	
ros2 run yolov8_object_detector object_detector
```
---

