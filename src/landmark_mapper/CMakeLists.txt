cmake_minimum_required(VERSION 3.0.2)
project(landmark_mapper)

## C++ Standard
add_compile_options(-std=c++17)

## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  roslib
)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(color_sampler
  src/color_sampler.cpp
)

add_dependencies(color_sampler ${catkin_EXPORTED_TARGETS})

add_executable(landmark_mapper_node
  src/landmark_mapper.cpp
)

add_dependencies(landmark_mapper_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(landmark_mapper_node
  color_sampler
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
  tf2
)


add_executable(color_sampler_node
  src/color_sampler_node.cpp
)


target_link_libraries(color_sampler_node
  color_sampler
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
  tf2
)

target_link_libraries(color_sampler
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)
