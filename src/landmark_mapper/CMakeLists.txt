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
  message_filters
  message_generation
)

## Find system libs
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

## Messages
add_message_files(
  FILES ColorSample.msg
)
generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
    image_transport
    message_filters
    message_runtime
  # DEPENDS nur für catkin-freie Exporte, hier nicht nötig
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## color_sampler node
add_executable(color_sampler_node src/color_sampler.cpp)
add_dependencies(color_sampler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(color_sampler_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

## landmark_mapper node
add_executable(landmark_mapper src/landmark_mapper.cpp)
add_dependencies(landmark_mapper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(landmark_mapper
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
