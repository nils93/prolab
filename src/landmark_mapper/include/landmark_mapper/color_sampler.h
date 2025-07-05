#ifndef LANDMARK_MAPPER_COLOR_SAMPLER_H
#define LANDMARK_MAPPER_COLOR_SAMPLER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include "landmark_mapper/ColorSample.h"

struct ColorSampleData {
  double rho, theta;
  std::string color;
};

class ColorSampler {
public:
  ColorSampler(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void saveSamples();
private:
  void callback(const sensor_msgs::ImageConstPtr& rgb,
                const sensor_msgs::ImageConstPtr& depth);
  std::string detectColor(const cv::Vec3b& bgr);
  ros::Publisher                             pub_;
  message_filters::Subscriber<sensor_msgs::Image> sub_rgb_, sub_depth_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync_;
  std::vector<ColorSampleData> samples_;
  std::string out_path_;
  ros::Timer   save_timer_;
};
#endif // LANDMARK_MAPPER_COLOR_SAMPLER_H
