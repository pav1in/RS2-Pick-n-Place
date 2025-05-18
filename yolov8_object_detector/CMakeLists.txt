cmake_minimum_required(VERSION 3.5)
project(yolov8_object_detector)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Tell ament to install your Python package
ament_python_install_package(${PROJECT_NAME})

ament_package()
