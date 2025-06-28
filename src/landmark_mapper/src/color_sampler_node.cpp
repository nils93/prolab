// color_sampler_node.cpp

#include "color_sampler_node/color_sampler_node.h"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <fstream>

ColorSampler::ColorSampler(ros::NodeHandle& nh)
: tf_listener_(tf_buffer_), last_save_time_(ros::Time(0)) {
  sub_image_ = nh.subscribe("/camera/image", 1, &ColorSampler::imageCallback, this);
}

void ColorSampler::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_img_ = cv_bridge::toCvCopy(msg, "bgr8")->image;

    if (!cv_img_.empty()) {
      ROS_INFO_ONCE("Erstes Bild empfangen: %d x %d", cv_img_.cols, cv_img_.rows);
    } else {
      ROS_WARN("Empfangenes Bild ist leer");
    }
  } catch (...) {
    ROS_ERROR("cv_bridge Fehler");
  }
}

void ColorSampler::addLandmark(double x, double y, double rho_or_radius, double theta, int type) {
  if ((ros::Time::now() - last_save_time_).toSec() < 1.0) return;

  std::array<int, 3> rgb = {255, 0, 0};  // fallback
  sampleColorAt(x, y, rgb);

  landmarks_.push_back({x, y, rho_or_radius, theta, type, rgb});
  writeYaml();
  last_save_time_ = ros::Time::now();
}

bool ColorSampler::sampleColorAt(double x, double y, std::array<int, 3>& rgb_out) {
  geometry_msgs::PointStamped map_pt, cam_pt;
  map_pt.header.frame_id = "map";
  map_pt.header.stamp = ros::Time(0);
  map_pt.point.x = x;
  map_pt.point.y = y;
  map_pt.point.z = 0.0;

  try {
    tf_buffer_.transform(map_pt, cam_pt, "camera_rgb_optical_frame", ros::Duration(1.0));
    double fx = 554.3827, fy = 554.3827, cx = 320.5, cy = 240.5;
    double X = cam_pt.point.x, Y = cam_pt.point.y, Z = cam_pt.point.z;

    if (Z <= 0) return false;

    int u = static_cast<int>(fx * X / Z + cx);
    int v = static_cast<int>(fy * Y / Z + cy);
    if (u >= 0 && u < cv_img_.cols && v >= 0 && v < cv_img_.rows) {
      cv::Vec3b color = cv_img_.at<cv::Vec3b>(v, u);
      rgb_out = {color[2], color[1], color[0]};
      return true;
    }
  } catch (...) {}
  return false;
}

void ColorSampler::writeYaml() {
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "landmarks" << YAML::Value << YAML::BeginMap;
  for (size_t i = 0; i < landmarks_.size(); ++i) {
    const auto& lm = landmarks_[i];
    out << YAML::Key << static_cast<int>(i + 1);
    out << YAML::Value << YAML::Flow << YAML::BeginSeq
        << lm.x << lm.y
        << YAML::Flow << YAML::BeginSeq << lm.rho_or_radius << lm.theta << lm.type
        << YAML::Flow << YAML::BeginSeq << lm.rgb[0] << lm.rgb[1] << lm.rgb[2]
        << YAML::EndSeq << YAML::EndSeq << YAML::EndSeq;
  }
  out << YAML::EndMap << YAML::EndMap;

  std::string path = ros::package::getPath("landmark_mapper") + "/landmark_map_colored.yaml";
  std::ofstream file(path);
  if (file.is_open()) {
    file << out.c_str();
    file.close();
    ROS_INFO("Landmark-Datei gespeichert unter: %s", path.c_str());
  } else {
    ROS_ERROR("Fehler beim Schreiben der Datei: %s", path.c_str());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "color_sampler_node");
  ros::NodeHandle nh;

  ColorSampler sampler(nh);

  ros::Rate rate(10); // 10 Hz
  ros::Time last_save = ros::Time::now();

  while (ros::ok()) {
    ros::spinOnce();

    // MARKIERT: Nur speichern, wenn seit dem letzten Speichern neue Daten kamen
    if ((ros::Time::now() - last_save).toSec() > 5.0 &&
        sampler.last_add_time_ > last_save) {
      std::string path = ros::package::getPath("landmark_mapper") + "/landmarks.yaml";
      sampler.writeYaml();
      ROS_INFO("Neue Landmarks gespeichert in %s", path.c_str());
      last_save = ros::Time::now();
    }

    rate.sleep();
  }

  return 0;
}