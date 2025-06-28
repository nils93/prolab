// color_sampler_node.h
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <array>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

struct Landmark {
  double x, y;
  double rho_or_radius;
  double theta;
  int type;
  std::array<int, 3> rgb;
};

class ColorSampler {
public:
  ColorSampler(ros::NodeHandle& nh);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void addLandmark(double x, double y, double rho_or_radius, double theta, int type);  // <-- extern aufrufbar

  ros::Time last_add_time_;  // MARKIERT: Zeitstempel des letzten addLandmark()
  void writeYaml();

private:
  bool sampleColorAt(double x, double y, std::array<int, 3>& rgb_out);
  

  ros::Subscriber sub_image_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  cv::Mat cv_img_;
  ros::Time last_save_time_;
  

  std::vector<Landmark> landmarks_;
};
