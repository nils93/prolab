// color_sampler_node.cpp

#include "color_sampler_node/color_sampler_node.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <fstream>

ColorSampler::ColorSampler(ros::NodeHandle& nh)
: nh_(nh), tf_buffer_(), tf_listener_(tf_buffer_) {
  sub_img_ = nh.subscribe("/camera/image_raw", 1, &ColorSampler::imageCallback, this);
  sub_lm_  = nh.subscribe("/landmark_debug", 1, &ColorSampler::landmarkCallback, this);
  last_save_time_ = ros::Time(0);  // <== MARKIERT
  loadLandmarks();
}

void ColorSampler::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge Exception: %s", e.what());
    return;
  }

  cv_img_ = cv_ptr->image.clone();  // optional: clone für Sicherheit
}

void ColorSampler::landmarkCallback(const visualization_msgs::MarkerArray::ConstPtr&) {
  if (cv_img_.empty()) return;
  if ((ros::Time::now() - last_save_time_).toSec() < 1.0) return;  // <== MARKIERT

  std::vector<Landmark> updated;

  for (const auto& lm : landmarks_) {
    geometry_msgs::PointStamped map_pt, cam_pt;
    map_pt.header.frame_id = "map";
    map_pt.header.stamp = ros::Time(0);
    map_pt.point.x = lm.x;
    map_pt.point.y = lm.y;
    map_pt.point.z = 0.0;

    try {
      tf_buffer_.transform(map_pt, cam_pt, "camera_rgb_optical_frame", ros::Duration(1.0));
      double fx = 554.3827128226441;
      double fy = 554.3827128226441;
      double cx = 320.5;
      double cy = 240.5;

      double X = cam_pt.point.x;
      double Y = cam_pt.point.y;
      double Z = cam_pt.point.z;

      if (Z <= 0) continue;

      int u = static_cast<int>(fx * X / Z + cx);
      int v = static_cast<int>(fy * Y / Z + cy);

      if (u >= 0 && u < cv_img_.cols && v >= 0 && v < cv_img_.rows) {
        cv::Vec3b color = cv_img_.at<cv::Vec3b>(v, u);
        Landmark new_lm = lm;
        new_lm.descriptor.push_back(color[2]); // R
        new_lm.descriptor.push_back(color[1]); // G
        new_lm.descriptor.push_back(color[0]); // B
        updated.push_back(new_lm);
        ROS_INFO("Landmark (%.2f, %.2f) → Farbe: [%d, %d, %d]",
                 lm.x, lm.y, color[2], color[1], color[0]);
      }
    } catch (tf2::TransformException& ex) {
      ROS_WARN("TF error: %s", ex.what());
    }
  }

  saveUpdated(updated);
  last_save_time_ = ros::Time::now();  // <== MARKIERT
}

void ColorSampler::loadLandmarks() {
  std::string path = ros::package::getPath("landmark_mapper") + "/landmark_map.yaml";
  YAML::Node doc = YAML::LoadFile(path);
  for (const auto& it : doc["landmarks"]) {
    Landmark lm;
    lm.x = it.second[0].as<double>();
    lm.y = it.second[1].as<double>();
    
    auto desc = it.second[2];
    lm.descriptor.push_back(desc[0].as<double>()); // rho
    lm.descriptor.push_back(desc[1].as<double>()); // theta
    lm.descriptor.push_back(desc[2].as<int>());    // type

    auto color = desc[3];
    lm.descriptor.push_back(color[0].as<int>());   // R
    lm.descriptor.push_back(color[1].as<int>());   // G
    lm.descriptor.push_back(color[2].as<int>());   // B

    landmarks_.push_back(lm);
  }
  ROS_INFO("%lu Landmarken geladen", landmarks_.size());
}

void ColorSampler::saveUpdated(const std::vector<Landmark>& updated) {
  std::string path = ros::package::getPath("landmark_mapper") + "/landmark_map_colored.yaml";
  std::ofstream out(path);
  out << "landmarks:\n";
  for (size_t i = 0; i < updated.size(); ++i) {
    const auto& lm = updated[i];
    out << "  " << i + 1 << ": [" << lm.x << ", " << lm.y;
    for (const auto& d : lm.descriptor)
      out << ", " << d;
    out << "]\n";
  }
  ROS_INFO("Landmarken mit Farben gespeichert → %s", path.c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "color_sampler_node");
  ros::NodeHandle nh;
  ColorSampler cs(nh);
  ros::spin();
  return 0;
}