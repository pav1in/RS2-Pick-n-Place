# WELCOME TO PICK N PLACE UR3e COBOT!
---
## Table of Contents

- [Installation](#installation)

---
## Installation

### Setting up Ubuntu 22.04 ROS 2 Software
Follow these steps on a clean Ubuntu 22.04 / ROS 2 Humble machine.
Skip any steps that you have already completed. Afterward, proceed to the subsystem setup for specific configurations.

1.	Install ROS 2 Humble Desktop-Full
```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions python3-pip
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

2.	Create and Initialize Your Colcon Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build         # (Initial empty build)
source install/setup.bash
```

3.	Link the Project Repository
```bash
Clone your project repository into a central folder (using SSH):
mkdir -p ~/git
cd ~/git
git clone git@github.com:pav1in/RS2-Pick-n-Place.git
```

4.	Symbolic Link to the Git Repository
Symlink your main package directory into your ROS 2 workspace for development:
```bash
cd ~/ros2_ws/src
ln -s ~/git/RS2-Pick-n-Place RS2-Pick-n-Place
```
5.	Build your colcon WS
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
---

### Fetch and Install Core Robotics Repositories

1. Clone the necessary driver and description repositories (use humble branches for ROS 2) to ~/ros2_ws/src
```bash
cd ~/ros2_ws/src
Universal Robots:
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
git clone https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
```

2. OnRobot (Gripper Support):
```bash
git clone https://github.com/tonydle/UR_OnRobot_ROS2.git ur_onrobot
cd ~/ros2_ws
vcs import src --input src/ur_onrobot/required.repos --recursive
cd ~/ros2_ws/src
```
The vcs import ... command will fetch additional dependencies such as onrobot_description and onrobot_driver.

3. ROS 2 Control Packages:
```bash
git clone -b humble https://github.com/ros-controls/ros2_control.git
git clone -b humble https://github.com/ros-controls/ros2_controllers.git
```
---
### Install GUI and Joint Control Tools
1. Handy GUI for testing joints:
```bash
sudo apt update
sudo apt install ros-humble-rqt ros-humble-rqt-joint-trajectory-controller
```
________________________________________
2. Resolve ROS 2 Dependencies and Build Everything
```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
---
