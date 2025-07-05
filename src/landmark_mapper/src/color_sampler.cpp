#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "landmark_mapper/color_sampler.h"
#include <filesystem>           // C++17
namespace fs = std::filesystem;

ColorSampler::ColorSampler(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: sub_rgb_(nh, "/camera/image",           1)
, sub_depth_(nh, "/camera/depth/image_raw",1)
, sync_(sub_rgb_, sub_depth_, 10)
{
  pnh.param<std::string>("output_path", out_path_, "landmarks_raw.yaml");
  ROS_INFO("ColorSampler: saving raw samples to '%s'", out_path_.c_str());
  {
    fs::path fp(out_path_);
    auto abs_fp = fs::absolute(fp);
    ROS_INFO("ColorSampler: saving raw samples to '%s' (abs: %s')",
             out_path_.c_str(), abs_fp.string().c_str());
  }
  pub_ = nh.advertise<landmark_mapper::ColorSample>("color_sample", 10);

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
            << s.rho << s.theta << s.color
            << YAML::EndSeq;
      }
      out << YAML::EndSeq;
      std::ofstream f(out_path_);
      f << out.c_str();
      ROS_INFO("Saved %lu samples to %s", samples_.size(), out_path_.c_str());
    }
  );
}


void ColorSampler::callback(const sensor_msgs::ImageConstPtr& rgb,
                            const sensor_msgs::ImageConstPtr& depth)
{
  // 1) Bilder in OpenCV konvertieren
  cv::Mat img = cv_bridge::toCvCopy(rgb, "bgr8")->image;
  cv::Mat dep = cv_bridge::toCvCopy(depth, depth->encoding)->image;
  cv::Mat hsv;
  cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

  // 2) Farben mit getunten HSV-Bereichen für Hydrant (rot) und Container (grün) + Kegel (orange)
  struct HSVRange {
    std::string name;
    int hue;    // Mittel-Hue
    int tol;    // +/- Toleranz um hue
    int s1,s2;  // Sättigung
    int v1,v2;  // Value
    bool requireCircle;  // nur kreisförmige Kontur?
  };
  std::vector<HSVRange> ranges = {
    // Hydrant: kräftiges Rot, kreisförmige Kontur nicht zwingend
    {"red",    0,  15, 100,255, 100,255, /*requireCircle=*/false},
    // Kegel: orange, kreisförmig
    {"orange",15,  10, 100,255, 100,255, /*requireCircle=*/false},
    // Container: dunkelgrün, keine Kreisform nötig
    {"green", 75,  30,  50,255,  50,255, /*requireCircle=*/false}
  };

  // 3) Für jede Farbe: Maske, Konturen, Tiefenprojektion
  for(auto& r : ranges){
    cv::Mat mask;

    // 3a) Hue-Bereich mit Wrap‐Around
    int h1 = (r.hue - r.tol + 180) % 180;
    int h2 = (r.hue + r.tol) % 180;
    if(h1 < h2){
      cv::inRange(hsv,
                  cv::Scalar(h1, r.s1, r.v1),
                  cv::Scalar(h2, r.s2, r.v2),
                  mask);
    } else {
      // Wrap‐around (z.B. rot)
      cv::Mat m1, m2;
      cv::inRange(hsv,
                  cv::Scalar(0,    r.s1, r.v1),
                  cv::Scalar(h2,   r.s2, r.v2),
                  m1);
      cv::inRange(hsv,
                  cv::Scalar(h1,   r.s1, r.v1),
                  cv::Scalar(179,  r.s2, r.v2),
                  m2);
      mask = m1 | m2;
    }

    // 4) Rauschen entfernen
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1,-1), 2);

    // 5) Konturen finden und filtern
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for(auto& c : contours){
      double area = cv::contourArea(c);
      if(area < 500) continue;  // zu klein

      // Kreis-Check
      cv::Point2f center; float radius;
      cv::minEnclosingCircle(c, center, radius);
      double circ = area / (M_PI * radius * radius);
      bool isCircle = (circ > 0.7);

      // Formabhängige Filterung
      if(r.requireCircle && !isCircle) continue;
      // für grün und rot erlauben wir beliebige Formen

      // 6) Tiefenwert auslesen
      float z = dep.at<float>(int(center.y), int(center.x));
      if(std::isnan(z) || z <= 0) continue;

      // 7) Polarkoordinaten berechnen
      double theta = atan2(center.x - rgb->width/2.0, 525.0);
      double rho   = z;

      // 8) Message füllen und publizieren
      landmark_mapper::ColorSample msg;
      msg.rho   = rho;
      msg.theta = theta;
      msg.color = r.name;
      pub_.publish(msg);

      // Roh-Sample speichern
      samples_.push_back({rho, theta, msg.color});
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
        << s.rho << s.theta << s.color
        << YAML::EndSeq;
  }
  out << YAML::EndSeq;

  fout << out.c_str();
  fout.close();
  ROS_INFO("Final save: %lu samples to %s",
           samples_.size(), out_path_.c_str());
}


// ----- main direkt hier -----
int main(int argc, char** argv){
  ros::init(argc, argv, "color_sampler_node");
  ros::NodeHandle nh;        // global für Topics
  ros::NodeHandle pnh("~");  // privat für Params
  ColorSampler sampler(nh, pnh);
  ros::spin();
  return 0;
}
