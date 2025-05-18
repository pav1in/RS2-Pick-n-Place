# Pick-n-Place ROS1

> **ROS Noetic** pick-and-place demo using UR3e, RealSense & YOLOv8  
> Tested on Ubuntu 20.04

---

## 📑 Table of Contents

- [Prerequisites](#-prerequisites)
- [Appendix A – Ubuntu 20.04 & ROS Noetic](#appendix-a--ubuntu-2004--ros-noetic)
  - [1. Install ROS Noetic](#1-install-ros-noetic)
  - [2. Create a Catkin Workspace](#2-create-a-catkin-workspace)
  - [3. Clone the Project](#3-clone-the-project)
  - [4. Link & Build](#4-link--build)
  - [5. Fetch UR Packages](#5-fetch-ur-packages)
  - [6. Install RQT Trajectory Plugin](#6-install-rqt-trajectory-plugin)
  - [7. Resolve Dependencies & Build](#7-resolve-dependencies--build)
- [Appendix B – Perception Subsystem](#appendix-b--perception-subsystem)
  - [Purpose](#purpose)
  - [Key ROS Topics](#key-ros-topics)
  - [SDK & ROS Wrapper](#sdk--ros-wrapper)
  - [OpenCV 4.6.0](#opencv-460)
  - [YOLOv8 (ONNX)](#yolov8-onnx)
  - [Simulation](#simulation)
  - [Real-life Demo](#real-life-demo)
- [Appendix C – Gripper Subsystem](#appendix-c--gripper-subsystem)
  - [Purpose](#purpose-1)
  - [Key ROS Topics](#key-ros-topics-1)
  - [Simulation Demo](#simulation-demo)
  - [Real-life Demo](#real-life-demo-1)
  - [Configurable Settings](#configurable-settings)
- [License](#license)

---

## 🔧 Prerequisites

- Ubuntu 20.04 LTS  
- ROS Noetic Desktop-full  
- Docker (optional for URSim)  
- Intel RealSense D435 / D455  
- Python 3.8+  

---

## Appendix A – Ubuntu 20.04 & ROS Noetic

### 1. Install ROS Noetic

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### 2. Create a Catkin Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Clone the Project
mkdir -p ~/git
cd ~/git
git clone git@github.com:pav1in/RS2-Pick-n-Place.git
cd RS2-Pick-n-Place
git checkout ros1
### 4. Link & Build

cd ~/catkin_ws/src
ln -s ~/git/RS2-Pick-n-Place RS2-Pick-n-Place
cd ~/catkin_ws
catkin_make
source devel/setup.bash
### 5. Fetch UR Packages

cd ~/catkin_ws/src
git clone https://github.com/ros-industrial/universal_robot.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
### 6. Install RQT Trajectory Plugin

sudo apt update
sudo apt install ros-noetic-rqt ros-noetic-rqt-joint-trajectory-controller
### 7. Resolve Dependencies & Build

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash

## Appendix B – Perception Subsystem
Purpose
Detects and classifies objects on the conveyor belt in real time using YOLOv8

Converts 2D detections to metric 3D poses using depth data and camera intrinsics

Publishes the filtered point cloud for collision layers and grasp planning

Provides a single source of truth via ROS topics

Key ROS Topics
Direction	Topic	Purpose
Subscribe	/camera/color/camera_info	Intrinsics & distortion parameters
Subscribe	/camera/color/image_raw	640×480 BGR frame for YOLOv8 detection
Subscribe	/camera/aligned_depth_to_color/image_raw	Depth image aligned to color
Subscribe	/camera/depth/color/points	Raw pointcloud for ROI segmentation
Publish	/detected_object_pose	6-DoF pose of each detected object
Publish	/segmented_roi	Pointcloud cropped to YOLO bounding boxes
Publish	/roi_marker	RViz marker showing each detected pose

SDK & ROS Wrapper

# 1. Add RealSense key & repo
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key 6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u
sudo apt-get update

# 2. Install librealsense2
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

# 3. Verify installation
realsense-viewer

# 4. Clone & build ROS wrapper
cd ~/catkin_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
OpenCV 4.6.0

cd ~
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd opencv && git checkout 4.6.0
cd ../opencv_contrib && git checkout 4.6.0
mkdir -p ~/opencv/build && cd ~/opencv/build
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
      -D OPENCV_GENERATE_PKGCONFIG=ON ..
make -j$(nproc)
sudo make install
sudo ldconfig
pkg-config --modversion opencv4  # expect 4.6.0

# Update environment variables
echo 'export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Rebuild CV Bridge for Noetic
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/vision_opencv.git
cd vision_opencv && git checkout -b noetic origin/noetic
cd ~/catkin_ws
catkin_make clean
catkin_make
source devel/setup.bash

#YOLOv8 (ONNX)
sudo apt install python3-pip
pip3 install ultralytics
# Convert your YOLOv8 .pt to ONNX once, then place in your package’s models/ folder.

#Simulation

cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch ur3e_simulation ur3e_simulation.launch
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
rosrun yolov8_detector_py yolov8_simulation.py

# Real-life Demo
cd ~/catkin_ws
source devel/setup.bash
roslaunch realsense2_camera rs_camera.launch serial_no:=317222073555 enable_pointcloud:=true align_depth:=true
rosrun yolov8_detector_py yolov8_pose_detector.py

## Appendix C – Gripper Subsystem
Purpose
Provides mechanical interface between UR3e & payloads

Executes open/close commands from grasp planner

Adaptive finger stroke: 0–110 mm

Publishes grip state & force feedback

Key ROS Topics
Direction	Topic	Purpose
Subscribe	/joint_states	Arm & gripper position, velocity, effort
Subscribe	/gripper_joint_position/command	Target finger position
Subscribe	/eff_joint_traj_controller/command	Homing & reposition commands
Publish	/gripper_force_estimate	Estimated closing force (Nm)
Publish	/ur_hardware_interface/set_io	Digital outputs for grip/release

## Simulation Demo

cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch ur3e_simulation ur3e_simulation.launch
roslaunch ur3_gripper_sim ur3_gripper_sim.launch
# Open gripper
rostopic pub /gripper_joint_position/command std_msgs/Float64 "data: -0.5" --once
# Close gripper
rostopic pub /gripper_joint_position/command std_msgs/Float64 "data: 0.0" --once
# Safety monitors
rosrun ur3_gripper_sim gripper_effort_monitor.py
rosrun ur3_gripper_sim gripper_force_limiter.py
Real-life Demo
bash
Copy
Edit
# Install RG2 URCap on teach pendant
# Mount gripper on UR3e
# Pendant: Installation → Tool: DO0=Grip, DO1=Release; Run → Program Tree → URCaps → External Control Node
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.0.101
# Grip
rosservice call /ur_hardware_interface/set_io "{fun:1, pin:0, state:1}"
# Release
rosservice call /ur_hardware_interface/set_io "{fun:1, pin:1, state:1}"
Configurable Settings
Setting	File/Location	Default	Description
Finger PID (position)	ur3_gripper_sim/config/position_controller.yaml	p:5.0, i:0.0, d:0.0	Finger stiffness
Physics PID	config/gazebo_controller.yaml	p:1.0, i:0.0, d:0.0	Gazebo control loop gains
Integral clamp & anti-windup	gazebo_controller.yaml	i_clamp:0.2	Prevent integrator wind-up
Max effort limit	gripper_force_limiter.py	2.2 Nm	Safety torque limit
Reset threshold	gripper_effort_monitor.py	<0.1 Nm	Effort below which limiter re-arms
Finger joint limits	ur3_gripper.urdf.xacro	-0.45 → 1.57 rad	Min/max finger spread
Finger speed	ur3_gripper.urdf.xacro	3.14 rad/s	Max closing speed


