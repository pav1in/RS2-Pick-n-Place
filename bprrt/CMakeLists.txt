######################################################################
## 1. Build flags & C++ standard
######################################################################
add_compile_options(-Wall -Wextra -Wno-unused-parameter -std=c++17)

# -------------------------------------------------------------------
# Decide ABI **once** and propagate to every target that needs Torch
# -------------------------------------------------------------------
#   * If you downloaded *_cxx11_abi*  ->  set TORCH_CXX11_ABI=1
#   * If you downloaded *_pre_cxx11_* ->  set TORCH_CXX11_ABI=0
#
# The variable can be exported from the shell, or we default to 1
# (which is what ROS Noetic, GCC‑9 and most modern packages use).
# -------------------------------------------------------------------
if(DEFINED ENV{TORCH_CXX11_ABI})
  set(TORCH_CXX11_ABI $ENV{TORCH_CXX11_ABI})
else()
  set(TORCH_CXX11_ABI 1)
endif()
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=${TORCH_CXX11_ABI})

######################################################################
## 2. Dependencies
######################################################################
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
  moveit_core moveit_ros_planning_interface pluginlib
)

# LibTorch -----------------------------------------------------------
find_package(Torch REQUIRED PATHS $ENV{CMAKE_PREFIX_PATH})
# (no ABI flags here – already handled globally)

######################################################################
## 3. catkin_package …   (unchanged)
######################################################################

######################################################################
## 5. Libraries
######################################################################
add_library(boxtracker src/boxtracker.cpp)
target_link_libraries(boxtracker ${catkin_LIBRARIES})
target_include_directories(boxtracker PUBLIC include)

add_library(bp_rrt_star_lib src/bp_rrt_star_lib.cpp)
target_link_libraries(bp_rrt_star_lib ${catkin_LIBRARIES} ${TORCH_LIBRARIES})
target_include_directories(bp_rrt_star_lib PUBLIC include)

######################################################################
## 6. Executable node
######################################################################
add_executable(bprrt_node src/bprrt_node.cpp)
target_link_libraries(bprrt_node
  bp_rrt_star_lib boxtracker
  ${catkin_LIBRARIES} ${TORCH_LIBRARIES}
)
target_include_directories(bprrt_node PUBLIC include)
