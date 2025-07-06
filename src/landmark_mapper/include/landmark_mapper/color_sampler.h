#ifndef LANDMARK_MAPPER_COLOR_SAMPLER_H
#define LANDMARK_MAPPER_COLOR_SAMPLER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <fstream>
#include <filesystem>
#include "landmark_mapper/ColorSample.h"
#include <apriltag_ros/AprilTagDetectionArray.h> // For AprilTag messages
#include <sensor_msgs/CameraInfo.h> // For camera intrinsics
#include <image_geometry/pinhole_camera_model.h> // For projecting 3D points
#include <opencv2/imgproc/imgproc.hpp>

struct ColorSampleData {
  double rho, theta;
  std::string color;
  int32_t tag_id;
};

// To store recently detected tags for correlation
struct RecentTag {
  int id;
  geometry_msgs::Point position; // 3D position from detection
  ros::Time timestamp;
};

// Struct to define a color to search for
struct ColorToTag {
  std::string name;
  int expected_tag_id;
  // HSV color range
  int h_mean, h_tol, s_min, s_max, v_min, v_max;
};

class ColorSampler {
public:
  ColorSampler(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void saveSamples();
private:
  void callback(const sensor_msgs::ImageConstPtr& rgb,
                const sensor_msgs::ImageConstPtr& depth);
  void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

  ros::Publisher pub_;
  ros::Subscriber sub_tags_; // Subscriber for AprilTags
  ros::Subscriber sub_cam_info_; // Subscriber for camera info
  message_filters::Subscriber<sensor_msgs::Image> sub_rgb_, sub_depth_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync_;
  
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<RecentTag> recent_tags_;
  std::vector<ColorToTag> colors_to_detect_;
  std::vector<ColorSampleData> samples_;
  std::string out_path_;
  ros::Timer   save_timer_;
};
#endif // LANDMARK_MAPPER_COLOR_SAMPLER_H
