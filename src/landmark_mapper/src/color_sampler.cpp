// color_sampler.cpp

#include "landmark_mapper/color_sampler.h"

ColorSampler::ColorSampler() : tf_listener_(tf_buffer_) {}

ColorSampler::ColorSampler(ros::NodeHandle& nh)
  : tf_listener_(tf_buffer_) {
  last_image_time_ = ros::Time(0);
  // Optional: Image-Subscriber hier einrichten
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

void ColorSampler::addLandmark(double x, double y, double rho, double theta, int type) {
  std::array<int, 3> rgb = {128, 128, 128};  // default
  if (sampleColorAt(x, y, rgb)) {
    landmarks_.emplace_back(x, y, rho, theta, type, rgb);
    ROS_INFO("Landmark mit Farbe hinzugefügt: (%.2f, %.2f) → [%d,%d,%d]", x, y, rgb[0], rgb[1], rgb[2]);
  } else {
    ROS_WARN("Farbstichprobe fehlgeschlagen bei (%.2f, %.2f)", x, y);
  }
  last_add_time_ = ros::Time::now();  // MARKIERT: Aktualisiere letzte Änderungszeit
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

    ROS_INFO("Transformiert zu Kamera: [%.2f, %.2f, %.2f], Bildgrösse: %d x %d, Alter: %.2f s",
           cam_pt.point.x, cam_pt.point.y, cam_pt.point.z,
           cv_img_.cols, cv_img_.rows,
           (ros::Time::now() - last_image_time_).toSec());

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
  } catch (tf2::TransformException& ex) {
    ROS_WARN("TF error: %s", ex.what());
  }
  return false;
}

void ColorSampler::saveToYAML(const std::string& path) {
  std::string out_path = path.empty()
      ? ros::package::getPath("landmark_mapper") + "/landmark_map_colored.yaml"
      : path;

  YAML::Emitter out;
  out << YAML::BeginMap << YAML::Key << "landmarks" << YAML::Value << YAML::BeginSeq;
  for (const auto& [x, y, rho, theta, type, rgb] : landmarks_) {
    out << YAML::Flow << YAML::BeginSeq << x << y
        << YAML::Flow << YAML::BeginSeq << rho << theta << type << YAML::Flow << YAML::BeginSeq << rgb[0] << rgb[1] << rgb[2] << YAML::EndSeq
        << YAML::EndSeq;
  }
  out << YAML::EndSeq << YAML::EndMap;

  std::ofstream fout(out_path);
  fout << out.c_str();
  ROS_INFO("Landmarks nach %s geschrieben", out_path.c_str());
}


