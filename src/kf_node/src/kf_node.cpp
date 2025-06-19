#include <ros/ros.h>
#include "kf_node/kf_localization.h"

int main(int argc, char** argv) {
  // Initialisiere ROS-Node mit Namen "kf_node"
  ros::init(argc, argv, "kf_node");

  // Erzeuge NodeHandle im privaten Namensraum (~)
  ros::NodeHandle nh("~");

  // Erzeuge EKF-Objekt, das sich um Subscriptions, Publikation und Berechnung kümmert
  KFLocalization kf(nh);

  // Starte ROS-Ereignisschleife
  ros::spin();

  return 0;
}
