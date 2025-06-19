#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcValue.h>
#include <Eigen/Dense>
#include <vector>
#include <random>

struct Landmark {
    int id;
    double x, y;
};

class PFLocalization {
public:
    PFLocalization(ros::NodeHandle& nh);
    
private:
    struct Particle {
        Eigen::Vector3d pose;  // x, y, theta
        double weight;
    };

    // Parameter
    std::vector<Landmark> LM_;
    std::vector<Particle> particles_;
    ros::Subscriber odom_sub_, lm_sub_;
    ros::Publisher pose_pub_;
    ros::Time last_time_;
    std::default_random_engine rng_;
    ros::Publisher particle_cloud_pub_;  // Neu: Publisher für Partikelwolke

    // Methoden
    void initParticles(int N);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void lmCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void predict(const nav_msgs::Odometry::ConstPtr& odom);
    void update(int id, const Eigen::Vector2d& z);
    void resample();
    void publishPose();
};

#endif // PARTICLE_FILTER_H