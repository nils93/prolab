#include "ekf.h"

// Globale Variablen
Eigen::VectorXd x_ekf;  // Zustand
Eigen::MatrixXd P_ekf;  // Kovarianz
Eigen::MatrixXd Q_ekf;  // Prozessrauschen
Eigen::MatrixXd R_ekf;  // Messrauschen

// Globale EKF-Variablen:
extern Eigen::VectorXd x_ekf;
extern Eigen::MatrixXd P_ekf;
extern Eigen::MatrixXd Q_ekf;
extern Eigen::MatrixXd R_ekf;
extern ros::Publisher pub_ekf_filter;

Eigen::VectorXd nonlinearStateTransition(const Eigen::VectorXd& x, double v, double omega) {
  Eigen::VectorXd x_pred = x;

  double dt = 0.1;  // Zeitschritt, z.B. 100 ms
  double theta = x(2);  // Aktuelle Orientierung

  // Bewegungsgleichung
  x_pred(0) += v * cos(theta) * dt;
  x_pred(1) += v * sin(theta) * dt;
  x_pred(2) += omega * dt;

  return x_pred;
}

Eigen::MatrixXd computeJacobianF(const Eigen::VectorXd& x) {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(3,3);

  double dt = 0.1;
  double v = 0.5;  // Dummy! Wird in nonlinearStateTransition übergeben
  double theta = x(2);

  F(0,2) = -v * sin(theta) * dt;
  F(1,2) =  v * cos(theta) * dt;

  return F;
}

Eigen::VectorXd nonlinearMeasurementFunction(const Eigen::VectorXd& x) {
  Eigen::VectorXd z_pred(3);
  z_pred << x(0), x(1), x(2);  // Direkte Messung
  return z_pred;
}

Eigen::MatrixXd computeJacobianH(const Eigen::VectorXd& x) {
  // Direkte Messung: Identity
  return Eigen::MatrixXd::Identity(3,3);
}

void ekfCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu) {
  double v = odom->twist.twist.linear.x; // Geschwindigkeit aus Odometry
  double omega = odom->twist.twist.angular.z; // Winkelgeschwindigkeit aus Odometry
  
  // Initialisierung
  Eigen::MatrixXd F_ekf = computeJacobianF(x_ekf);
  Eigen::MatrixXd H_ekf = computeJacobianH(x_ekf);

  // Predict
  x_ekf = nonlinearStateTransition(x_ekf, v, omega);
  P_ekf = F_ekf * P_ekf * F_ekf.transpose() + Q_ekf;

  // Messung z
  Eigen::VectorXd z(3);
  z << odom->pose.pose.position.x, odom->pose.pose.position.y, imu->angular_velocity.z;

  // Update
  Eigen::VectorXd y = z - nonlinearMeasurementFunction(x_ekf);
  Eigen::MatrixXd S = H_ekf * P_ekf * H_ekf.transpose() + R_ekf;
  Eigen::MatrixXd K = P_ekf * H_ekf.transpose() * S.inverse();
  x_ekf = x_ekf + K * y;
  P_ekf = (Eigen::MatrixXd::Identity(3,3) - K * H_ekf) * P_ekf;

  // Publish gefilterte Odometry
  nav_msgs::Odometry filtered_msg = *odom;
  filtered_msg.pose.pose.position.x = x_ekf(0);
  filtered_msg.pose.pose.position.y = x_ekf(1);
  pub_ekf_filter.publish(filtered_msg);

}
