#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "landmark_mapper/landmark_mapper.h"
#include <filesystem>
namespace fs = std::filesystem;

LandmarkMapper::LandmarkMapper(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: tf_ls_(tf_buf_) 
{
  // Parameter aus privatem Handle
  pnh.param<std::string>("output_path", out_path_, "mapped_landmarks.yaml");
  ROS_INFO("LandmarkMapper: saving landmarks to '%s'", out_path_.c_str());
  // Subscriber auf globales Topic
  sub_   = nh.subscribe("color_sample", 10, &LandmarkMapper::sampleCb, this);
  // Timer zum periodischen Speichern
  timer_ = nh.createTimer(ros::Duration(5.0), &LandmarkMapper::saveCb, this);
}

void LandmarkMapper::sampleCb(const landmark_mapper::ColorSample::ConstPtr& msg){
  geometry_msgs::TransformStamped tfst;
  try{
    tfst = tf_buf_.lookupTransform("map","base_link",ros::Time(0),ros::Duration(0.1));
  } catch(...){ return; }
  double xr = tfst.transform.translation.x;
  double yr = tfst.transform.translation.y;
  tf2::Quaternion q;
  tf2::fromMsg(tfst.transform.rotation, q);

  // Yaw extrahieren
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double x = xr + msg->rho * cos(yaw + msg->theta);
  double y = yr + msg->rho * sin(yaw + msg->theta);

  for(auto& lm:data_){
    if(fabs(lm.x-x)<0.1 && fabs(lm.y-y)<0.1 && lm.color==msg->color) return;
  }
  data_.push_back({x,y,msg->rho,msg->theta,msg->color});
}

void LandmarkMapper::saveCb(const ros::TimerEvent&){
  YAML::Emitter out;
  out<<YAML::BeginSeq;
  for(auto& lm:data_){
    out<<YAML::Flow<<YAML::BeginSeq
       <<lm.x<<lm.y<<lm.rho<<lm.theta<<lm.color
       <<YAML::EndSeq;
  }
  out<<YAML::EndSeq;
  std::ofstream f(out_path_);
  f<<out.c_str();
  ROS_INFO("Wrote %lu landmarks", data_.size());
}

void LandmarkMapper::saveLandmarks(){
  fs::path fp(out_path_);
  if(fp.has_parent_path()){
    fs::create_directories(fp.parent_path());
  }
  std::ofstream fout(out_path_, std::ios::out | std::ios::trunc);
  if(!fout.is_open()){
    ROS_ERROR("Cannot open file '%s' for writing", out_path_.c_str());
    return;
  }
  YAML::Emitter out;
  out << YAML::BeginSeq;
  for(const auto& lm : data_){
    out << YAML::Flow << YAML::BeginSeq
        << lm.x << lm.y
        << YAML::BeginSeq << lm.rho << lm.theta << lm.color << YAML::EndSeq
        << YAML::EndSeq;
  }
  out << YAML::EndSeq;

  fout << out.c_str();
  fout.close();
  ROS_INFO("Final save: %zu landmarks to %s",
           data_.size(), out_path_.c_str());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "landmark_mapper");
  ros::NodeHandle nh, pnh("~");
  LandmarkMapper lm(nh, pnh);

  ros::spin();

  // Nach Ctrl-C: letzte Speicherung
  lm.saveLandmarks();
  return 0;
}
