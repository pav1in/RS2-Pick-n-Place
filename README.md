# Collision Avoidance Subsystem - Library

This branch provides a calculation library for use with the trajectory subsystem, also providing an example test use case. 

## Purpose

Collect object data into a 27-cell "workspace" in order to calculate weightings to influence the trajectory planning's algorithm.

---

## Table of Contents

- [Purpose](#purpose)
- [Installation and Setup](#installation-and-setup)

---

## Installation and Setup

### 1. Cloning the git:
Make sure you have an SSH key setup for your Ubuntu installation before running:
```bash
git clone git@github.com:pav1in/RS2-Pick-n-Place.git
```

### 2. Installing dependencies:
 First, make sure all your programs and package installer are up to date:
 ```bash
 sudo apt-get update
 sudo apt-get upgrade
 ```

### >> IMPORTANT <<
Make sure you have g++ >= v. 11.4.0 and CMake >= v. 3.22

 Then you can install the dependencies (CGAL and CGAL QT5):
 ```bash
 sudo apt-get install libcgal-dev libcgal-qt5-dev
 ```

### 3. Switch to the Collision-avoidance branch:
```bash
cd ~/path/to/RS2-Pick-n-Place
git checkout Collision-avoidance
```

### 4. Building the library:
#### 4a. Navigating to the library folder:
```bash
cd ~/path/to/the/RS2-Pick-n-Place/collision-avoid
e.g. cd ~/ros2_ws/src/RS2-Pick-n-Place/collision_avoid
```
#### 4b. Building with CMake:
There should be a build folder already in the library folder, but if there isn't:
```bash
cmake -B build
```
In the same library folder as before:
```bash
cmake --build build
```
#### 4c. Installing library into CMake:
```bash
cmake --install build
```
If that returns a ```Permission Denied``` error, you will need to run it as an admin:
```bash
sudo cmake --install build
```

## >> OPTIONAL << 
If you would like to see an example visual representation of the library working, please so the following:
### Building and running the test file 
Navigate to the folder:
```bash
cd ~/path/to/the/RS2-Pick-n-Place/collision_test/build
e.g. cd ~/ros2_ws/src/RS2-Pick-n-Place/collision_test/build
```
Use CMake to build the test package:
```bash
cmake ..
cmake –build .
```
Then you can run the output file:
```bash
./collision_test
```

You can view the final output of probabilities in the terminal, as well as meshes in the ```meshes``` folder located in ```collision_test```



---
