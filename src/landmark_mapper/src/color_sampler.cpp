#include "landmark_mapper/color_sampler.h"
  
namespace fs = std::filesystem;

ColorSampler::ColorSampler(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: sub_rgb_(nh, "/camera/image",           1)
, sub_depth_(nh, "/camera/depth/image_raw",1)
, sync_(sub_rgb_, sub_depth_, 10)
{
  pnh.param<std::string>("output_path", out_path_, "landmarks_raw.yaml");
  ROS_INFO("ColorSampler: output path parameter is '%s'", out_path_.c_str());
  ROS_INFO("ColorSampler: full output path is '%s'", fs::absolute(out_path_).c_str());

  pub_ = nh.advertise<landmark_mapper::ColorSample>("color_sample", 10);

  // Define the colors and their corresponding tag IDs to detect
  // HSV values: H (0-179), S (0-255), V (0-255)
  colors_to_detect_ = {
    {"green",  0, 60, 15, 70, 255, 70, 255},
    {"red",    1, 0,  10, 70, 255, 70, 255}, // Red wraps around 0/180
    {"blue",   2, 120,15, 70, 255, 70, 255},
    {"orange", 3, 15, 10, 100,255, 100,255} // Orange is around H=15-25
  };

  // Subscribe to camera info to get calibration
  sub_cam_info_ = nh.subscribe("/camera/camera_info", 1, &ColorSampler::cameraInfoCallback, this);

  // Subscribe to AprilTag detections
  sub_tags_ = nh.subscribe("/tag_detections", 10, &ColorSampler::tagCallback, this);

  sync_.registerCallback(
    boost::bind(&ColorSampler::callback, this, _1, _2)
  );

  save_timer_ = nh.createTimer(
    ros::Duration(5.0),
    [&](const ros::TimerEvent&){
      YAML::Emitter out;
      out << YAML::BeginSeq;
      for (auto& s : samples_) {
        out << YAML::Flow << YAML::BeginSeq
            << s.rho << s.theta
            << YAML::Flow << YAML::BeginSeq << s.color << s.tag_id << YAML::EndSeq
            << YAML::EndSeq;
      }
      out << YAML::EndSeq;
      std::ofstream f(out_path_);
      f << out.c_str();
      ROS_INFO("Saved %lu samples to %s", samples_.size(), out_path_.c_str());
    }
  );
}

void ColorSampler::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg){
  if(!cam_model_.initialized()){
    cam_model_.fromCameraInfo(msg);
    ROS_INFO("ColorSampler: Camera model initialized.");
    sub_cam_info_.shutdown();
  }
}

void ColorSampler::tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  // 1. Lösche alte tags, die älter als 1 Sekunde sind
  ros::Time now = ros::Time::now();
  recent_tags_.erase(
    std::remove_if(recent_tags_.begin(), recent_tags_.end(),
                   [&](const RecentTag& t){ return (now - t.timestamp).toSec() > 1.0; }),
    recent_tags_.end());

  // 2. Füge neue Tags hinzu
  for(const auto& detection : msg->detections)
  {
    recent_tags_.push_back({
      detection.id[0],
      detection.pose.pose.pose.position,
      detection.pose.header.stamp
    });
  }
}


void ColorSampler::callback(const sensor_msgs::ImageConstPtr& rgb,
                            const sensor_msgs::ImageConstPtr& depth)
{
  if(!cam_model_.initialized()){
    return;
  }

  // 1) Konvertiere Bilder zu OpenCV
  cv::Mat img = cv_bridge::toCvCopy(rgb, "bgr8")->image;
  cv::Mat dep = cv_bridge::toCvCopy(depth, depth->encoding)->image;
  cv::Mat hsv;
  cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

  // 2) Bereite die HSV-Bilder vor
  for (const auto& ctt : colors_to_detect_) {
    cv::Mat mask;
    if (ctt.name == "red") {
      cv::Mat mask1, mask2;
      cv::inRange(hsv, cv::Scalar(0, ctt.s_min, ctt.v_min), cv::Scalar(ctt.h_tol, ctt.s_max, ctt.v_max), mask1);
      cv::inRange(hsv, cv::Scalar(180 - ctt.h_tol, ctt.s_min, ctt.v_min), cv::Scalar(179, ctt.s_max, ctt.v_max), mask2);
      mask = mask1 | mask2;
    } else {
      int h1 = (ctt.h_mean - ctt.h_tol + 180) % 180;
      int h2 = (ctt.h_mean + ctt.h_tol) % 180;
      cv::inRange(hsv, cv::Scalar(h1, ctt.s_min, ctt.v_min), cv::Scalar(h2, ctt.s_max, ctt.v_max), mask);
    }

    // 3) clean up
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1,-1), 2);

    // 4) Find color contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for(auto& c : contours){
      double area = cv::contourArea(c);
      if(area < 500) continue;  // zu kleine Flächen ignorieren

      // 5) Finde den Mittelpunkt der farbigen Region
      cv::Moments M = cv::moments(c);
      cv::Point2f color_center(M.m10 / M.m00, M.m01 / M.m00);

      // 6) Prüfe, ob ein passender AprilTag vorhanden ist
      const RecentTag* matched_tag = nullptr; // Use a pointer to store the matched tag
      for(const auto& tag : recent_tags_){
        if(tag.id != ctt.expected_tag_id) continue;
        // Project 3D tag position to 2D image coordinates to check for proximity
        cv::Point3d tag_pos_3d(tag.position.x, tag.position.y, tag.position.z);
        cv::Point2d tag_pos_2d = cam_model_.project3dToPixel(tag_pos_3d);
        // Check if the projected tag center is close to the color blob center
        if(cv::norm(color_center - cv::Point2f(tag_pos_2d)) < 25.0) { // 25 pixel tolerance - Erfahrungswert
          matched_tag = &tag; // Store a pointer to the matched tag
          break; 
        }
      }

      // 7) Falls sowohl Farbe als auch Tag übereinstimmen, erstelle und veröffentliche die Probe
      if(matched_tag){
        cv::Point3d point_in_cam_frame(
            matched_tag->position.x,
            matched_tag->position.y,
            matched_tag->position.z
        );

        double x_base = point_in_cam_frame.z;
        double y_base = -point_in_cam_frame.x;
        double rho   = std::sqrt(x_base * x_base + y_base * y_base);
        double theta = std::atan2(y_base, x_base);
        landmark_mapper::ColorSample sample_msg;
        sample_msg.header.stamp = rgb->header.stamp;
        sample_msg.header.frame_id = "base_link";
        sample_msg.rho = rho;
        sample_msg.theta = theta;
        sample_msg.color = ctt.name;
        sample_msg.tag_id = ctt.expected_tag_id;
        pub_.publish(sample_msg);
        samples_.push_back({rho, theta, ctt.name, ctt.expected_tag_id});

        break; 
      }
    }
  }
}

void ColorSampler::saveSamples(){
  // 1) Verzeichnis anlegen, falls nötig
  fs::path fp(out_path_);
  if(fp.has_parent_path()){
    fs::create_directories(fp.parent_path());
  }
  // 2) Datei öffnen (trunc = neu anlegen)
  std::ofstream fout(out_path_, std::ios::out | std::ios::trunc);
  if(!fout.is_open()){
    ROS_ERROR("Cannot open file '%s' for writing", out_path_.c_str());
    return;
  }
  // 3) YAML schreiben
  YAML::Emitter out;
  out << YAML::BeginSeq;
  for(const auto& s : samples_){
    out << YAML::Flow << YAML::BeginSeq
        << s.rho << s.theta
        << YAML::Flow << YAML::BeginSeq << s.color << s.tag_id << YAML::EndSeq
        << YAML::EndSeq;
  }
  out << YAML::EndSeq;

  fout << out.c_str();
  fout.close();
  ROS_INFO("Final save: %lu samples to %s",
           samples_.size(), out_path_.c_str());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "color_sampler_node");
  ros::NodeHandle nh;        
  ros::NodeHandle pnh("~");
  ColorSampler sampler(nh, pnh);
  ros::spin();
  // Nach Ctrl-C: letzte Speicherung
  sampler.saveSamples();
  return 0;
}
