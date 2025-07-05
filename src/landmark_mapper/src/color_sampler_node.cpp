#include <ros/ros.h>
#include "landmark_mapper/color_sampler.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "color_sampler_node");
  ros::NodeHandle nh, pnh("~");
  ColorSampler sampler(nh, pnh);

  ros::spin();

  // Nach Ctrl-C: letzte Speicherung
  sampler.saveSamples();
  return 0;
}
