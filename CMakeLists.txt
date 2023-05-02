cmake_minimum_required(VERSION 3.10.2)
project(rpo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set (octomap_DIR "/opt/ros/melodic/share/octomap/")

find_package(OpenMP REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  octomap
  octomap_ros
  octomap_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(RPO
  src/main.cpp
  src/custom_octree.cpp
)

target_link_libraries(RPO
  ${catkin_LIBRARIES}
  OpenMP::OpenMP_CXX
)

