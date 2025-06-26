// waypoint_nav.h
#ifndef YOUR_PACKAGE_WAYPOINT_NAV_H
#define YOUR_PACKAGE_WAYPOINT_NAV_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <tuple>

struct Waypoint {
  double x;
  double y;
};

class WaypointNav {
public:
  WaypointNav(ros::NodeHandle& nh);
  int run();

private:
  bool loadWaypoints(const std::string& filename);

  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
  std::vector<Waypoint> waypoints_;
  std::vector<std::tuple<int,bool,ros::Time>> summary_;
};

#endif // YOUR_PACKAGE_WAYPOINT_NAV_H
