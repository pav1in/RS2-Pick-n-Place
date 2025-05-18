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


Symlink only the yolov8_object_detector package into your ROS 2 workspace:

bash
Copy
Edit
cd ~/ros2_ws/src
ln -s ~/git/RS2-Pick-n-Place/yolov8_object_detector yolov8_object_detector
If you switch branches, remove the symlink before linking a different version.

2. Install Required Python Packages
bash
Copy
Edit
pip3 install --user ultralytics onnx onnxruntime ros2_numpy opencv-python
3. Install Required ROS 2 Packages
bash
Copy
Edit
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-tf-transformations
sudo apt install ros-humble-librealsense2* ros-humble-realsense2-camera
4. Install and Configure the RealSense ROS 2 Wrapper
bash
Copy
Edit
sudo usermod -a -G dialout $USER
wget -O ~/99-realsense.rules https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
sudo mv ~/99-realsense.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
Note: You may need to log out and back in for group permissions to take effect.

5. Build and Source the Workspace
bash
Copy
Edit
cd ~/ros2_ws
colcon build --symlink-install --packages-select yolov8_object_detector
source install/setup.bash
Subsystem Demonstration (Real Hardware)
Terminal A: Launch the RealSense Node
bash
Copy
Edit
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  pointcloud.enable:=true \
  depth_module.profile:=640x480x30
Terminal B: Run the YOLOv8 Object Detector Node
bash
Copy
Edit
cd ~/ros2_ws
colcon build --symlink-install --packages-select yolov8_object_detector
source install/setup.bash
ros2 run yolov8_object_detector object_detector
Terminal C: Launch RViz and Visualize Topics
bash
Copy
Edit
rviz2
RViz Visualization Topics
In RViz, add the following displays and set the corresponding topics:

PointCloud2: /camera/depth/color/points

PointCloud2: /segmented_roi (set Color Transformer: RGB8)

Marker: /roi_marker

Marker: /obb_marker

Image: /yolo/image

(Optional: TF for visualizing frames)

Troubleshooting
USB/Permission Issues:
Add your user to the dialout group and configure udev rules (see installation step 4).

No RealSense topics:
Ensure the camera is plugged into a USB 3.0 port and all dependencies are installed.

No detections or poor results:
Adjust detection thresholds in object_detector.py or check your camera alignment.

This perception subsystem provides all necessary 2D/3D detection outputs for integration with your pick-and-place pipeline.
For further support, refer to the Issues section or contact the repository maintaine



