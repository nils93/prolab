#include "kf.h"

Eigen::VectorXd x_kf;  // Zustand
Eigen::MatrixXd P_kf;  // Kovarianz
Eigen::MatrixXd F_kf;  // Zustandsübergang
Eigen::MatrixXd Q_kf;  // Prozessrauschen
Eigen::MatrixXd H_kf;  // Messmatrix
Eigen::MatrixXd R_kf;  // Messrauschen

extern ros::Publisher pub_kf_filter;



void kfCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu) {
  // Beispiel-Update: Messung aus Odometry + IMU extrahieren
  Eigen::VectorXd z(3);
  z << odom->pose.pose.position.x, odom->pose.pose.position.y, imu->angular_velocity.z;
 
  // DEBUG
  // Debug: Ausgabe Dimensionen
  //ROS_INFO_STREAM("z dim: " << z.size());
  //ROS_INFO_STREAM("H_kf rows: " << H_kf.rows() << " cols: " << H_kf.cols());
  //ROS_INFO_STREAM("x_kf dim: " << x_kf.size());
  // Debug: Ausgabe Produkt-Dimension
  Eigen::VectorXd Hx = H_kf * x_kf;
  //ROS_INFO_STREAM("H_kf * x_kf dim: " << Hx.size());
  // DEBUG

  // Predict
  x_kf = F_kf * x_kf;
  P_kf = F_kf * P_kf * F_kf.transpose() + Q_kf;

  // Update
  Eigen::VectorXd y = z - H_kf * x_kf;
  Eigen::MatrixXd S = H_kf * P_kf * H_kf.transpose() + R_kf;
  Eigen::MatrixXd K = P_kf * H_kf.transpose() * S.inverse();
  x_kf = x_kf + K * y;
  P_kf = (Eigen::MatrixXd::Identity(x_kf.size(), x_kf.size()) - K * H_kf) * P_kf;

  // Publish Ergebnis
  nav_msgs::Odometry filtered_msg = *odom;
  filtered_msg.pose.pose.position.x = x_kf(0);
  filtered_msg.pose.pose.position.y = x_kf(1);
  pub_kf_filter.publish(filtered_msg);

}
