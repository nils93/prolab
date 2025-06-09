#include "kf.h"

// Kalman Filter State (siehe Kap. 3.4, Gleichung 3.18ff)
Eigen::VectorXd x_kf;  // Zustand: [x, y, theta]
Eigen::MatrixXd P_kf;  // Kovarianzmatrix
Eigen::MatrixXd F_kf;  // Übergangsmatrix (motion model linearisiert)
Eigen::MatrixXd Q_kf;  // Prozessrauschen
Eigen::MatrixXd H_kf;  // Beobachtungsmatrix (measurement model linearisiert)
Eigen::MatrixXd R_kf;  // Messrauschen

extern ros::Publisher pub_kf_filter;

void kfCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu) {
  // Kapitel 3.4: z = H * x + δ (Messung linear zum Zustand)

  // z: Beobachtung (z. B. Position aus Odometrie, Winkelgeschwindigkeit aus IMU)
  Eigen::VectorXd z(3);
  z << odom->pose.pose.position.x,
       odom->pose.pose.position.y,
       imu->angular_velocity.z;

  // 1. Prädiktion (Prediction Step) – Gleichung 3.18, 3.19
  x_kf = F_kf * x_kf;
  P_kf = F_kf * P_kf * F_kf.transpose() + Q_kf;

  // 2. Korrektur (Correction Step) – Gleichung 3.20 bis 3.23
  Eigen::VectorXd y = z - H_kf * x_kf;                               // Innovationsvektor
  Eigen::MatrixXd S = H_kf * P_kf * H_kf.transpose() + R_kf;        // Innovationskovarianz
  Eigen::MatrixXd K = P_kf * H_kf.transpose() * S.inverse();        // Kalman Gain

  x_kf = x_kf + K * y;                                              // Zustandskorrektur
  P_kf = (Eigen::MatrixXd::Identity(x_kf.size(), x_kf.size()) - K * H_kf) * P_kf;  // Kovarianz-Update

  // Ergebnis publizieren
  nav_msgs::Odometry filtered_msg = *odom;
  filtered_msg.pose.pose.position.x = x_kf(0);
  filtered_msg.pose.pose.position.y = x_kf(1);
  pub_kf_filter.publish(filtered_msg);
}
