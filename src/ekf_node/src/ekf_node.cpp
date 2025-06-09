#include <ros/ros.h>
#include "ekf_node/ekf_localization.h"

int main(int argc, char** argv) {
  // Initialisiere ROS-Node mit Namen "ekf_node"
  ros::init(argc, argv, "ekf_node");

  // Erzeuge NodeHandle im privaten Namensraum (~)
  ros::NodeHandle nh("~");

  // Erzeuge EKF-Objekt, das sich um Subscriptions, Publikation und Berechnung kümmert
  EKFLocalization ekf(nh);

  // Starte ROS-Ereignisschleife
  ros::spin();

  return 0;
}
