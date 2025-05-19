# Perception Subsystem – ROS 2 (YOLOv8 + RealSense D435 Integration)

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
## Hardware

| No. | Category | Item                                     | Qty | Purpose                                        | Notes/Part No.                         |
|-----|----------|------------------------------------------|-----|------------------------------------------------|----------------------------------------|
| 1   | Camera   | Intel RealSense Depth Camera D435        | 1   | Colour + depth sensing for object detection & point cloud | Refer to (Intel RealSense, 2025)       |
| 2   | Camera   | Articulated tripod / desk stand          | 1   | Allows strategic positioning                   | Refer to (Intel RealSense, 2025)       |
| 3   | Camera   | USB-C to USB-A 3.0 cable                 | 1   | To connect camera to computer                  | Refer to (Intel RealSense, 2025)       |
| 4   | Camera   | Calibration checkerboard (8×6, 20 mm)    | 1   | Eye-to-hand extrinsic calibration              | 8×6 inner corners, 20 mm squares       |



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

## Troubleshooting & FAQs

### 1. Experiencing USB Permission Issues?

| Step | Command / Action                                                                                                                                                                                                                                                     | Description                                                                  |
|------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------|
| 1    | ```bash<br>sudo usermod -a -G dialout $USER<br>```                                                                                                                                                                                                                     | Add your user to the dialout group to grant serial and USB device access.    |
| 2    | ```bash<br>sudo reboot<br>```                                                                                                                                                                                                                                          | Logout/login or reboot (especially on WSL or VM) to apply group changes.     |
| 3    | ```bash<br>sudo wget -O /etc/udev/rules.d/99-realsense-libusb.rules https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules<br>```                                       | Download and install Intel RealSense udev rules so camera permissions load automatically. |
| 4    | ```bash<br>sudo udevadm control --reload-rules && sudo udevadm trigger<br>```                                                                                                                                                                                           | Reload udev rules and trigger them immediately.                              |


---

### 2. Issues with Camera Display or OpenCV GUI?

You may see errors or black windows in RViz, `rqt_image_view`, or other OpenCV-based GUIs—especially inside a VM.

| Step | Action                                                                                                                                                                                                                                                                                       | Details                                                                                                                                                   |
|------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1    | **Switch to Xorg**<br>- At Ubuntu’s login screen, click the ⚙️ (gear) icon or session menu near the password field.<br>- Select **“Ubuntu on Xorg”** (or “Ubuntu on X11”), then log in.                                                                                                       | Wayland-based sessions often block or misrender GUI windows in VMs.                                                                                       |
| 2    | **Enable Xorg in GDM (if no gear icon)**<br>- Edit `/etc/gdm3/custom.conf` as root.<br>- Uncomment the line:<br>  ```ini<br>  #WaylandEnable=false<br>  ```<br>- Save and reboot.                                                                                                            | Forces GDM to disable Wayland and use Xorg by default.                                                                                                   |
| 3    | **Verify your session type**<br>```bash<br>echo $XDG_SESSION_TYPE<br>```                                                                                                                                                                                                                       | Should print `x11`. If it prints `wayland`, log out and redo step 1.                                                                                       |


---
