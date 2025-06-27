// color_sampler_node.h

#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <opencv2/core.hpp>

struct Landmark {
  double x, y;
  std::vector<double> descriptor;
};

class ColorSampler {
public:
  ColorSampler(ros::NodeHandle& nh);

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void landmarkCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void loadLandmarks();
  void saveUpdated(const std::vector<Landmark>& updated);

  ros::NodeHandle nh_;
  ros::Subscriber sub_img_;
  ros::Subscriber sub_lm_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<Landmark> landmarks_;
  cv::Mat cv_img_;
  ros::Time last_save_time_;
};
