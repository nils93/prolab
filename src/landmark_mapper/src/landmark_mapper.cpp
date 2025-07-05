#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "landmark_mapper/landmark_mapper.h"
#include <filesystem>
#include <ros/package.h> // Hinzufügen für ros::package::getPath
namespace fs = std::filesystem;

LandmarkMapper::LandmarkMapper(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: tf_ls_(tf_buf_) 
{
  // Parameter aus privatem Handle
  pnh.param<std::string>("output_path", out_path_, "mapped_landmarks.yaml");
  
  ROS_INFO("LandmarkMapper: output path parameter is '%s'", out_path_.c_str());
  ROS_INFO("LandmarkMapper: full output path is '%s'", fs::absolute(out_path_).c_str());

  // Subscriber auf globales Topic
  sub_   = nh.subscribe("color_sample", 10, &LandmarkMapper::sampleCb, this);
  // Timer zum periodischen Speichern
  timer_ = nh.createTimer(ros::Duration(5.0), &LandmarkMapper::saveCb, this);
}

void LandmarkMapper::sampleCb(const landmark_mapper::ColorSample::ConstPtr& msg){
  ROS_INFO_ONCE("LandmarkMapper: Received first color sample.");
  geometry_msgs::TransformStamped tfst;
  try{
    tfst = tf_buf_.lookupTransform("map","base_link",ros::Time(0),ros::Duration(0.1));
  } catch(tf2::TransformException &ex){
    ROS_WARN("Could not get transform from map to base_link: %s", ex.what());
    return;
  }
  double xr = tfst.transform.translation.x;
  double yr = tfst.transform.translation.y;
  tf2::Quaternion q;
  tf2::fromMsg(tfst.transform.rotation, q);

  // Yaw extrahieren
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double x = xr + msg->rho * cos(yaw + msg->theta);
  double y = yr + msg->rho * sin(yaw + msg->theta);

  // Landmark-Signatur ist die Kombination aus Farbe und Tag-ID
  for (auto& lm : data_) {
    if (lm.color == msg->color && lm.tag_id == msg->tag_id) {
      // Landmark mit derselben Signatur gefunden, Position aktualisieren
      lm.x = x;
      lm.y = y;
      lm.rho = msg->rho;
      lm.theta = msg->theta;
      ROS_DEBUG("Updated landmark [\"%s\", %d] at (%f, %f)", lm.color.c_str(), lm.tag_id, x, y);
      return; // Fertig
    }
  }

  // Kein passendes Landmark gefunden, neues hinzufügen
  data_.push_back({x, y, msg->rho, msg->theta, msg->color, msg->tag_id});
  ROS_INFO("Added new landmark [\"%s\", %d] at (%f, %f)", msg->color.c_str(), msg->tag_id, x, y);
}

void LandmarkMapper::writeFile(bool is_final){
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
        << YAML::BeginSeq << lm.color << lm.tag_id << YAML::EndSeq
        << YAML::EndSeq;
  }
  out << YAML::EndSeq;

  fout << out.c_str();
  fout.close();
  if(is_final){
    ROS_INFO("Final save: %zu landmarks to %s",
           data_.size(), out_path_.c_str());
  } else {
    ROS_INFO("Wrote %lu landmarks to %s", data_.size(), out_path_.c_str());
  }
}

void LandmarkMapper::saveCb(const ros::TimerEvent&){
  writeFile(false);
}

void LandmarkMapper::saveLandmarks(){
  writeFile(true);
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
