cmake_minimum_required(VERSION 3.0.2)
project(fastslam)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  fiducial_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  waveshare_alphabot2
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fastslam
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs fiducial_msgs tf2 tf2_ros tf2_geometry_msgs waveshare_alphabot2
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(particle_filter
  src/particle_filter.cpp
)

## Declare a C++ executable
add_executable(slam_node src/slam.cpp)

## Add dependencies to the executable
add_dependencies(slam_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(particle_filter
  ${catkin_LIBRARIES}
)

target_link_libraries(slam_node
  particle_filter
  ${catkin_LIBRARIES}
)
