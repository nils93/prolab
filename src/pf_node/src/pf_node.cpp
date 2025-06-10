#include "pf_node/pf_localization.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
  // Initialisiere ROS-Node mit Namen "pf_node"
  ros::init(argc, argv, "pf_node");

  // Erzeuge NodeHandle im privaten Namensraum (~)
  ros::NodeHandle nh("~");

  // Erzeuge PF-Objekt, das sich um Subscriptions, Publikation und Berechnung kümmert
  PFLocalization pf(nh);

  // Starte ROS-Ereignisschleife
  ros::spin();

  return 0;
}
