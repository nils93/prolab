#ifndef LANDMARK_MAPPER_LANDMARK_MAPPER_H
#define LANDMARK_MAPPER_LANDMARK_MAPPER_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <string>
#include "landmark_mapper/ColorSample.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // für fromMsg()
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

struct Landmark {
  double x,y, rho, theta;
  std::string color;
  int32_t tag_id;
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
