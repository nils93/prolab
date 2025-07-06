#ifndef EKF_WLM_H
#define EKF_WLM_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <landmark_mapper/ColorSample.h>

struct Landmark {
    double x, y;
    std::string color;
    int id;
};

class EKFLocalizationWLM {
public:
    EKFLocalizationWLM(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void landmarkCallback(const landmark_mapper::ColorSample::ConstPtr& msg);
    void predict(const nav_msgs::Odometry::ConstPtr& odom);
    void updateWithLandmark(const landmark_mapper::ColorSample::ConstPtr& msg, const Landmark& lm);
    void publishPose();
    bool loadLandmarks(const std::string& path);

    // ROS
    ros::Subscriber odom_sub_;
    ros::Subscriber landmark_sub_;
    ros::Publisher pose_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // EKF State
    Eigen::Vector3d mu_;          // [x, y, theta]
    Eigen::Matrix3d Sigma_;       // Covariance matrix
    Eigen::Matrix3d R_;           // Process noise
    Eigen::Matrix2d Q_landmark_;  // Measurement noise (landmarks)

    // Landmarks
    std::vector<Landmark> landmark_map_;
    ros::Time last_time_;
};

#endif // EKF_WLM_H