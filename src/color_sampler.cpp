#include <ros/ros.h>
#include <landmark_mapper/ColorSample.h>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

class ColorSampler
{
public:
  ColorSampler(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("output_path", out_path_, "landmarks_raw.yaml");
    ROS_INFO("ColorSampler: output path parameter is '%s'", out_path_.c_str());
    ROS_INFO("ColorSampler: full output path is '%s'", fs::absolute(out_path_).c_str());

    pub_ = nh.advertise<landmark_mapper::ColorSample>("color_sample", 10);
  }

private:
  ros::Publisher pub_;
  std::string out_path_;
};