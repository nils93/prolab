#ifndef EKF_WLM_H
#define EKF_WLM_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "landmark_mapper/ColorSample.h" // Für die Landmarken-Messung

// Struktur zur Speicherung der Landmarken aus der Karte
struct Landmark {
    double x;
    double y;
    std::string color;
    int id;
};

class EKFLocalizationWLM {
public:
    EKFLocalizationWLM(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    // Callbacks
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void landmarkCallback(const landmark_mapper::ColorSample::ConstPtr& msg);

    // EKF-Schritte
    void predict(const nav_msgs::Odometry::ConstPtr& odom);
    void update(const landmark_mapper::ColorSample::ConstPtr& msg);

    // Hilfsfunktionen
    void publishPose();
    bool loadLandmarks(const std::string& path);

    // ROS-Kommunikation
    ros::Subscriber odom_sub_;
    ros::Subscriber landmark_sub_;
    ros::Publisher pose_pub_;

    // Zustand und Kovarianz
    Eigen::Vector3d mu_;      // Zustand [x, y, yaw]
    Eigen::Matrix3d Sigma_;   // Kovarianz
    Eigen::Matrix3d R_;       // Prozessrauschen
    Eigen::Matrix2d Q_;       // Messrauschen (für rho, theta)

    // Landmarken-Karte
    std::vector<Landmark> landmark_map_;

    // Zeitstempel für dt-Berechnung
    ros::Time last_time_;
};

#endif // EKF_WLM_H
