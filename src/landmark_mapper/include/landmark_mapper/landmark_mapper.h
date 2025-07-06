#ifndef LANDMARK_MAPPER_LANDMARK_MAPPER_H
#define LANDMARK_MAPPER_LANDMARK_MAPPER_H

#include "landmark_mapper/ColorSample.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // für fromMsg()
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>

struct Landmark {
  double x,y, rho, theta;
  std::string color;
  int32_t tag_id;
  int detection_count; // Zählt, wie oft die Landmarke gesehen wurde
};

class LandmarkMapper {
public:
  LandmarkMapper(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void saveLandmarks();
private:
  void sampleCb(const landmark_mapper::ColorSample::ConstPtr& msg);
  void saveCb(const ros::TimerEvent&);
  void writeFile(bool is_final);
  
  ros::Subscriber    sub_;
  ros::Timer         timer_;
  tf2_ros::Buffer    tf_buf_;
  tf2_ros::TransformListener tf_ls_;
  std::vector<Landmark> data_;
  std::string out_path_;
};

#endif // LANDMARK_MAPPER_LANDMARK_MAPPER_H
