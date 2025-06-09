#include "ekf.h"


Eigen::Vector2d latest_scan_measurement;
bool got_scan = false;

// EKF-Zustandsgrößen
Eigen::VectorXd x_ekf;  // [x, y, theta]
Eigen::MatrixXd P_ekf;
Eigen::MatrixXd Q_ekf;
Eigen::MatrixXd R_ekf;
std::vector<Eigen::Vector2d> landmark_map;  // Landmarkpositionen [l1, l2, ...]
extern ros::Publisher pub_ekf_filter;

Eigen::VectorXd nonlinearStateTransition(const Eigen::VectorXd& x, double v, double omega) {
  double dt = 0.1;
  double theta = x(2);
  Eigen::VectorXd x_pred = x;

  x_pred(0) += v * cos(theta) * dt;
  x_pred(1) += v * sin(theta) * dt;
  x_pred(2) += omega * dt;

  return x_pred;
}

Eigen::MatrixXd computeJacobianF(const Eigen::VectorXd& x, double v) {
  double dt = 0.1;
  double theta = x(2);
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(3,3);
  F(0,2) = -v * sin(theta) * dt;
  F(1,2) =  v * cos(theta) * dt;
  return F;
}

Eigen::Vector2d expectedMeasurement(const Eigen::VectorXd& x, const Eigen::Vector2d& landmark) {
  double dx = landmark(0) - x(0);
  double dy = landmark(1) - x(1);
  double range = sqrt(dx*dx + dy*dy);
  double bearing = atan2(dy, dx) - x(2);
  return Eigen::Vector2d(range, bearing);
}

Eigen::MatrixXd computeJacobianH(const Eigen::VectorXd& x, const Eigen::Vector2d& landmark) {
  double dx = landmark(0) - x(0);
  double dy = landmark(1) - x(1);
  double q = dx*dx + dy*dy;
  double sqrt_q = sqrt(q);

  Eigen::MatrixXd H(2,3);
  H(0,0) = -dx / sqrt_q;
  H(0,1) = -dy / sqrt_q;
  H(0,2) = 0;
  H(1,0) = dy / q;
  H(1,1) = -dx / q;
  H(1,2) = -1;
  return H;
}

void ekfCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu) {
  double v = odom->twist.twist.linear.x;
  double omega = odom->twist.twist.angular.z;

  // Prediction
  x_ekf = nonlinearStateTransition(x_ekf, v, omega);
  Eigen::MatrixXd F = computeJacobianF(x_ekf, v);
  P_ekf = F * P_ekf * F.transpose() + Q_ekf;

  if (!got_scan) return;

  Eigen::Vector2d z = latest_scan_measurement;

  double best_score = std::numeric_limits<double>::infinity();
  int best_index = -1;
  Eigen::Vector2d best_y;
  Eigen::MatrixXd best_H;
  Eigen::Matrix2d best_S;

  for (int i = 0; i < landmark_map.size(); ++i) {
    const Eigen::Vector2d& landmark = landmark_map[i];
    double dx = landmark(0) - x_ekf(0);
    double dy = landmark(1) - x_ekf(1);
    double q = dx * dx + dy * dy;
    if (q < 1e-6) continue;

    Eigen::Vector2d z_pred;
    z_pred(0) = std::sqrt(q);
    z_pred(1) = std::atan2(dy, dx) - x_ekf(2);

    Eigen::Vector2d y = z - z_pred;
    y(1) = atan2(std::sin(y(1)), std::cos(y(1)));

    Eigen::MatrixXd H(2,3);
    H(0,0) = -dx / std::sqrt(q);
    H(0,1) = -dy / std::sqrt(q);
    H(0,2) = 0;
    H(1,0) = dy / q;
    H(1,1) = -dx / q;
    H(1,2) = -1;

    Eigen::Matrix2d S = H * P_ekf * H.transpose() + R_ekf.topLeftCorner(2,2);
    double score = y.transpose() * S.inverse() * y;

    if (score < best_score) {
      best_score = score;
      best_index = i;
      best_y = y;
      best_H = H;
      best_S = S;
    }
  }

  if (best_index >= 0) {
    Eigen::MatrixXd K = P_ekf * best_H.transpose() * best_S.inverse();
    x_ekf = x_ekf + K * best_y;
    P_ekf = (Eigen::MatrixXd::Identity(3,3) - K * best_H) * P_ekf;
    ROS_INFO_STREAM_THROTTLE(1.0, "Landmark " << best_index << " verwendet bei EKF-Update, x = " << x_ekf.transpose());
  }

  got_scan = false;

  // === tf-Broadcast: map → base_footprint ===
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = ros::Time::now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "base_footprint";
  tf_msg.transform.translation.x = x_ekf(0);
  tf_msg.transform.translation.y = x_ekf(1);
  tf_msg.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, x_ekf(2));
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  br.sendTransform(tf_msg);

  // === Odometry-Output ===
  nav_msgs::Odometry filtered_msg = *odom;
  filtered_msg.header.frame_id = "map";
  filtered_msg.child_frame_id = "base_footprint";
  filtered_msg.pose.pose.position.x = x_ekf(0);
  filtered_msg.pose.pose.position.y = x_ekf(1);

  filtered_msg.pose.pose.orientation.x = q.x();
  filtered_msg.pose.pose.orientation.y = q.y();
  filtered_msg.pose.pose.orientation.z = q.z();
  filtered_msg.pose.pose.orientation.w = q.w();

  pub_ekf_filter.publish(filtered_msg);
}


void loadLandmarks(const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("Fehler beim Öffnen von " << filepath);
    return;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    double x, y;
    if (iss >> x >> y) {
      landmark_map.emplace_back(x, y);
    }
  }

  ROS_INFO_STREAM("Geladene Landmarken: " << landmark_map.size());
}

void scanCallback(const sensor_msgs::LaserScanConstPtr& scan) {
  float min_range = std::numeric_limits<float>::infinity();
  int min_index = -1;

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    float r = scan->ranges[i];
    if (std::isfinite(r) && r < min_range) {
      min_range = r;
      min_index = i;
    }
  }

  if (min_index >= 0) {
    float angle = scan->angle_min + min_index * scan->angle_increment;
    latest_scan_measurement << min_range, angle;
    got_scan = true;
  }
}

