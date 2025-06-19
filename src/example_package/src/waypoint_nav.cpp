// waypoint_nav.cpp
#include "waypoint_nav.h"
#include <ros/ros.h>
#include <tf/tf.h>

WaypointNav::WaypointNav(ros::NodeHandle& nh)
: nh_(nh),
  ac_("move_base", true)
{
  ac_.waitForServer();
}

bool WaypointNav::loadWaypoints(const std::string& filename) {
  YAML::Node doc = YAML::LoadFile(filename);
  for (auto node : doc["waypoints"]) {
    waypoints_.push_back({ node["x"].as<double>(),
                           node["y"].as<double>() });
  }
  ROS_INFO("Loaded %zu waypoints", waypoints_.size());
  return true;
}

int WaypointNav::run() {
  std::string fn;
  nh_.param<std::string>("waypoints_file", fn, "");
  if (!loadWaypoints(fn)) return 1;

  // ### Hinzugefügter Code-Snippet START ###
  double delay_sec = 0.0;
  nh_.param("start_delay", delay_sec, 0.0);
  ROS_INFO("waypoint_nav: start_delay = %.2f seconds", delay_sec);
  if (delay_sec > 0.0) {
    ROS_INFO("Waiting %.2f seconds before starting navigation", delay_sec);
    ros::Duration(delay_sec).sleep();
  } else {
    ROS_INFO("Press ENTER to start navigation...");
    std::cin.get();
  }
  // ### Hinzugefügter Code-Snippet ENDE ###

  for (size_t i = 0; i < waypoints_.size(); ++i) {
    const auto& wp = waypoints_[i];
    ROS_INFO("Goto waypoint %zu: x=%.2f y=%.2f", i, wp.x, wp.y);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp    = ros::Time::now();
    goal.target_pose.pose.position.x = wp.x;
    goal.target_pose.pose.position.y = wp.y;
    // Identity-Quaternion: keine Vorgabe der Orientierung
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    ac_.sendGoal(goal);
    bool ok = ac_.waitForResult(ros::Duration(60.0)) &&
              ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    summary_.emplace_back(i, ok, ros::Time::now());
    ROS_INFO("Waypoint %zu %s", i, ok ? "erreicht" : "fehlgeschlagen");
  }

  ROS_INFO("Fahre alle Waypoints ab. Zusammenfassung:");
  for (auto& e : summary_) {
    int idx; bool ok; ros::Time ts;
    std::tie(idx, ok, ts) = e;
    ROS_INFO(" %d: %s um %.2f", idx, ok ? "OK" : "FAIL", ts.toSec());
  }
  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_nav");
  ros::NodeHandle nh("~");
  return WaypointNav(nh).run();
}
