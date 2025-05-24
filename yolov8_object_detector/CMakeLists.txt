# CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(yolov8_object_detector)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# only CMake‐available ROS packages here:
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
# ← removed: find_package(ament_index_python REQUIRED)
# ← removed: find_package(ros2_numpy REQUIRED)

# install your Python module for `import yolov8_object_detector`
ament_python_install_package(${PROJECT_NAME})

# install the node script so `ros2 run` finds it
install(
  PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/yolov8_object_detector/object_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# install all models/ recursively
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# install resource/ and package.xml
install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}/resource
)
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
