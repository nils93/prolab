#ifndef EKF_NODE_EKF_LOCALIZATION_H
#define EKF_NODE_EKF_LOCALIZATION_H

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Eigen für lineare Algebra
#include <Eigen/Dense>
#include <vector>

// Struktur zur Beschreibung einer Landmarke (ID + Position)
struct Landmark {
  int id;   // eindeutige Kennung
  double x, y; // Position in der Karte
};

// Klasse zur Implementierung eines Extended Kalman Filters (EKF)
class EKFLocalization {
public:
  // Konstruktor mit ROS-NodeHandle
  EKFLocalization(ros::NodeHandle& nh);

private:
  // ROS Subscriber und Publisher
  ros::Subscriber odom_sub_;  // für Odometrie-Eingang
  ros::Subscriber lm_sub_;    // für Landmarkenbeobachtungen
  ros::Publisher  pose_pub_;  // veröffentlichte geschätzte Pose

  // EKF-Zustand und Kovarianzmatrizen
  Eigen::Vector3d x_;      // Zustand: [x, y, theta]
  Eigen::Matrix3d P_;      // Kovarianzmatrix des Zustands
  Eigen::Matrix3d Q_;      // Prozessrauschen
  Eigen::Matrix2d R_;      // Messrauschen

  std::vector<Landmark> LM_; // Liste bekannter Landmarken
  ros::Time last_time_;      // letzter Zeitstempel für die Zeitschrittdifferenz

  // Callback-Funktionen für ROS-Nachrichten
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);                         // Vorhersageschritt
  void lmCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);    // Korrekturschritt

  // EKF-Schritte
  void predict(const nav_msgs::Odometry::ConstPtr& odom);               // Bewegungsvorhersage
  void update(int id, const Eigen::Vector2d& z);                        // Korrektur mit Landmarkenbeobachtung
  void publishPose();                                                   // Ausgabe der geschätzten Pose als ROS-Topic
};

#endif // EKF_NODE_EKF_LOCALIZATION_H
