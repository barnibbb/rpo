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

add_executable(Parameters
  test/parameters_test.cpp
  src/parameters.cpp
)

target_link_libraries(Parameters
  ${catkin_LIBRARIES}
)

add_executable(Preprocessor
  test/preprocessor_test.cpp
  src/augmented_model.cpp
  src/parameters.cpp
)

target_link_libraries(Preprocessor
  ${catkin_LIBRARIES}
)

add_executable(Calculator
  test/calculator_test.cpp
  src/dose_calculator.cpp
  src/augmented_octree.cpp
  src/parameters.cpp
)

target_link_libraries(Calculator
  ${catkin_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_executable(Visualizer
  test/visualizer_test.cpp
  src/ros_visualizer.cpp
  src/dose_calculator.cpp
  src/augmented_octree.cpp
  src/parameters.cpp
)

target_link_libraries(Visualizer
  ${catkin_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_executable(Test
  test/augmented_octree_test.cpp
  src/augmented_octree.cpp
)

target_link_libraries(Test
  ${catkin_LIBRARIES}
)



#add_executable(RPO
#  src/main.cpp
#  src/extended_octree.cpp
#  src/augmented_model.cpp
#)

# target_link_libraries(RPO
#   ${catkin_LIBRARIES}
#   OpenMP::OpenMP_CXX
# )

