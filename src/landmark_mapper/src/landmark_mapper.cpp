// landmark_mapper.cpp

#include "landmark_mapper/landmark_mapper.h"
#include "color_sampler_node/color_sampler_node.h"

LandmarkMapper::LandmarkMapper(ros::NodeHandle& nh)
: nh_(nh), tfBuffer_(), tfListener_(tfBuffer_) {
  sub_scan_ = nh_.subscribe("scan", 1, &LandmarkMapper::scanCallback, this);
  ROS_INFO("LandmarkMapper gestartet");
}

void LandmarkMapper::setColorSampler(ColorSampler* cs) {
  cs_ = cs;
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
  //ROS_INFO("ScanCallback erhalten: %lu Punkte", points.size());
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

  const int threshold =20;                // vorher 100
  for (int t = 0; t < width; ++t) {
    for (int r = 0; r < height; ++r) {
      if (accumulator[t][r] > threshold) {
        double theta = t * theta_res;
        double rho = r * rho_res - 10.0;
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;

        //ROS_INFO("Hough-Line detected: theta=%.2f, ρ=%.2f, votes=%d", theta, rho, accumulator[t][r]);

        geometry_msgs::PointStamped ps, ps_out;
        ps.header.stamp = ros::Time(0);
        ps.header.frame_id = "base_link";
        ps.point.x = x0;
        ps.point.y = y0;

        try {
          tfBuffer_.transform(ps, ps_out, "map", ros::Duration(1.0));
          Eigen::Vector2d pos(ps_out.point.x, ps_out.point.y);
          bool known = false;
          for (const auto& k : known_positions_) {
            if ((pos - k).norm() < 0.3) {
              known = true;
              break;
            }
          }
          if (known) continue;

          known_positions_.push_back(pos);
          publishLandmark(pos.x(), pos.y(), rho, theta, 0);
        } catch (...) {}
      }
    }
  }
}

void LandmarkMapper::detectCircle(const std::vector<Eigen::Vector2d>& points) {
  if (points.size() < 10) return;

  const int max_iter = 100;
  const double thresh = 0.1;
  const int min_inliers = 5;

  std::default_random_engine gen;
  std::uniform_int_distribution<size_t> dist(0, points.size() - 1);

  for (int iter = 0; iter < max_iter; ++iter) {
    size_t i1 = dist(gen), i2 = dist(gen), i3 = dist(gen);
    if (i1 == i2 || i1 == i3 || i2 == i3) continue;

    const auto& A = points[i1], B = points[i2], C = points[i3];

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

    //ROS_INFO("Circle candidate: center=(%.2f, %.2f), r=%.2f, inliers=%d", center.x(), center.y(), radius, inliers);


    if (inliers >= min_inliers) {
      geometry_msgs::PointStamped ps, ps_out;
      ps.header.stamp = ros::Time(0);
      ps.header.frame_id = "base_link";
      ps.point.x = center.x();
      ps.point.y = center.y();

      try {
        tfBuffer_.transform(ps, ps_out, "map", ros::Duration(1.0));
        Eigen::Vector2d pos(ps_out.point.x, ps_out.point.y);
        bool known = false;
        for (const auto& k : known_positions_) {
          if ((pos - k).norm() < 0.3) {
            known = true;
            break;
          }
        }
        if (known) continue;

        known_positions_.push_back(pos);
        publishLandmark(pos.x(), pos.y(), radius, 0.0, 1);
        return;
      } catch (...) {}
    }
  }
}

void LandmarkMapper::publishLandmark(double x, double y, double rho_or_radius, double theta, int type) {
  if (cs_) {
    cs_->addLandmark(x, y, rho_or_radius, theta, type);
    ROS_INFO("Landmark published: (%.2f, %.2f), type=%d", x, y, type);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "landmark_mapper_node");
  ros::NodeHandle nh;

  LandmarkMapper mapper(nh);
  ColorSampler sampler(nh);
  mapper.setColorSampler(&sampler);  // << WICHTIG

  ros::spin();
  return 0;
}


