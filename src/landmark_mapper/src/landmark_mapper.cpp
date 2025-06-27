// landmark_mapper.cpp

#include "landmark_mapper/landmark_mapper.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <random>
#include <cmath>

LandmarkMapper::LandmarkMapper(ros::NodeHandle& nh)
: nh_(nh), tfBuffer_(), tf_listener_(tfBuffer_)
{
  sub_ = nh.subscribe("scan", 1, &LandmarkMapper::scanCallback, this);
  pub_ = nh.advertise<visualization_msgs::Marker>("landmark_debug", 1);
  ROS_INFO("LandmarkMapper gestartet");
}

void LandmarkMapper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  std::vector<Eigen::Vector2d> points;
  double angle = msg->angle_min;

  for (const auto& r : msg->ranges) {
    if (r > msg->range_min && r < msg->range_max) {
      double x = r * std::cos(angle);
      double y = r * std::sin(angle);
      points.emplace_back(x, y);
    }
    angle += msg->angle_increment;
  }

  detectHoughLines(points);
  detectCircle(points);

  visualization_msgs::Marker points_marker;
  points_marker.header.frame_id = "base_link";
  points_marker.header.stamp = ros::Time::now();
  points_marker.ns = "raw_points";
  points_marker.id = 0;
  points_marker.type = visualization_msgs::Marker::POINTS;
  points_marker.scale.x = 0.03;
  points_marker.scale.y = 0.03;
  points_marker.color.r = 1.0;
  points_marker.color.g = 1.0;
  points_marker.color.b = 1.0;
  points_marker.color.a = 1.0;

  for (const auto& p : points) {
    geometry_msgs::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = 0.0;
    points_marker.points.push_back(pt);
  }

  pub_.publish(points_marker);
}

void LandmarkMapper::detectHoughLines(const std::vector<Eigen::Vector2d>& points) {
  const double theta_res = M_PI / 180.0;
  const double rho_res = 0.05;
  const int width = 180;
  const int height = 400;
  std::vector<std::vector<int>> accumulator(width, std::vector<int>(height, 0));

  for (const auto& pt : points) {
    double x = pt.x();
    double y = pt.y();
    for (int t = 0; t < width; ++t) {
      double theta = t * theta_res;
      double rho = x * cos(theta) + y * sin(theta);
      int r_idx = static_cast<int>((rho + 10.0) / rho_res);
      if (r_idx >= 0 && r_idx < height) {
        accumulator[t][r_idx]++;
      }
    }
  }

  const int threshold = 100;
  for (int t = 0; t < width; ++t) {
    for (int r = 0; r < height; ++r) {
      if (accumulator[t][r] > threshold) {
        double theta = t * theta_res;
        double rho = r * rho_res - 10.0;
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;

        geometry_msgs::PointStamped ps, ps_out;
        ps.header.stamp = ros::Time(0);
        ps.header.frame_id = "base_link";
        ps.point.x = x0;
        ps.point.y = y0;
        ps.point.z = 0.0;

        try {
          tfBuffer_.transform(ps, ps_out, "map", ros::Duration(1.0));
          for (const auto& lm : raw_landmarks_) {
            if (std::hypot(lm.x - ps_out.point.x, lm.y - ps_out.point.y) < 0.3)
              return;
          }
          Landmark lm;
          lm.x = ps_out.point.x;
          lm.y = ps_out.point.y;
          lm.rho_or_radius = rho;
          lm.theta = theta;
          lm.type = 0;
          lm.rgb = getSimulatedColor(lm.x, lm.y);
          raw_landmarks_.push_back(lm);
          ROS_INFO("🔵 Linien-LM gespeichert: (%.2f, %.2f)", lm.x, lm.y);
        } catch (tf2::TransformException& ex) {
          ROS_WARN("TF fehlgeschlagen: %s", ex.what());
        }
      }
    }
  }
}

void LandmarkMapper::detectCircle(const std::vector<Eigen::Vector2d>& points) {
  if (points.size() < 10) return;

  const int max_iter = 100;
  const double thresh = 0.05;
  const int min_inliers = 10;

  std::default_random_engine gen;
  std::uniform_int_distribution<size_t> dist(0, points.size() - 1);

  for (int iter = 0; iter < max_iter; ++iter) {
    size_t i1 = dist(gen), i2 = dist(gen), i3 = dist(gen);
    if (i1 == i2 || i1 == i3 || i2 == i3) continue;

    const auto& A = points[i1];
    const auto& B = points[i2];
    const auto& C = points[i3];

    double a = 2 * (B.x() - A.x()), b = 2 * (B.y() - A.y());
    double c = 2 * (C.x() - A.x()), d = 2 * (C.y() - A.y());
    double det = a * d - b * c;
    if (std::abs(det) < 1e-6) continue;

    double u = ((B.squaredNorm() - A.squaredNorm()) * d - (C.squaredNorm() - A.squaredNorm()) * b) / det;
    double v = ((C.squaredNorm() - A.squaredNorm()) * a - (B.squaredNorm() - A.squaredNorm()) * c) / det;
    Eigen::Vector2d center(u, v);
    double radius = (center - A).norm();

    int inliers = 0;
    for (const auto& p : points)
      if (std::abs((p - center).norm() - radius) < thresh)
        inliers++;

    if (inliers >= min_inliers) {
      geometry_msgs::PointStamped ps, ps_out;
      ps.header.stamp = ros::Time(0);
      ps.header.frame_id = "base_link";
      ps.point.x = center.x();
      ps.point.y = center.y();
      ps.point.z = 0.0;

      try {
        tfBuffer_.transform(ps, ps_out, "map", ros::Duration(1.0));
        for (const auto& lm : raw_landmarks_) {
          if (std::hypot(lm.x - ps_out.point.x, lm.y - ps_out.point.y) < 0.3)
            return;
        }
        Landmark lm;
        lm.x = ps_out.point.x;
        lm.y = ps_out.point.y;
        lm.rho_or_radius = radius;
        lm.theta = 0.0;
        lm.type = 1;
        lm.rgb = getSimulatedColor(lm.x, lm.y);
        raw_landmarks_.push_back(lm);
        ROS_INFO("🟠 Kreis-LM gespeichert: (%.2f, %.2f), r=%.2f", lm.x, lm.y, radius);
        return;
      } catch (tf2::TransformException& ex) {
        ROS_WARN("TF fehlgeschlagen: %s", ex.what());
      }
    }
  }
}

void LandmarkMapper::finalizeLandmarks() {
  saveLandmarks();
}

void LandmarkMapper::saveLandmarks() {
  std::string path = ros::package::getPath("landmark_mapper") + "/landmark_map.yaml";
  std::ofstream out(path);
  out << "landmarks:\n";
  for (size_t i = 0; i < raw_landmarks_.size(); ++i) {
    const auto& lm = raw_landmarks_[i];
    out << "  " << i + 1 << ": [" << lm.x << ", " << lm.y << ", [" << lm.rho_or_radius << ", " << lm.theta << ", " << lm.type
        << ", [" << lm.rgb[0] << ", " << lm.rgb[1] << ", " << lm.rgb[2] << "]]]\n";
  }
  ROS_INFO("📝 %lu Landmarken gespeichert: %s", raw_landmarks_.size(), path.c_str());
}

std::array<int, 3> LandmarkMapper::getSimulatedColor(double x, double y) {
  // 🧪 Simulierter Farbrückgabewert – später durch Kamera-Callback ersetzen
  if (x < -2.0)
    return {255, 0, 0};  // rot
  if (y < -6.0)
    return {0, 255, 0};  // grün
  return {0, 0, 255};    // blau
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "landmark_mapper");
  ros::NodeHandle nh;
  LandmarkMapper mapper(nh);
  ros::spin();
  mapper.finalizeLandmarks();
  return 0;

  
}