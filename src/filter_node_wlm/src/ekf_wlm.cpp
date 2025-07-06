#include "ekf_wlm/ekf_wlm.h"
#include <yaml-cpp/yaml.h>
#include <fstream>

EKFLocalizationWLM::EKFLocalizationWLM(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    // Load landmarks from YAML file
    std::string landmark_path;
    pnh.param<std::string>("landmark_map_path", landmark_path, "");
    
    if (!loadLandmarks(landmark_path)) {
        ROS_ERROR("Failed to load landmarks from %s", landmark_path.c_str());
        ros::shutdown();
        return;
    }

    // Initialize EKF
    mu_.setZero();
    Sigma_ = Eigen::Matrix3d::Identity() * 0.1;
    R_ = Eigen::Matrix3d::Identity() * 1e-3;
    Q_landmark_ = Eigen::Matrix2d::Identity() * 1e-4;
    last_time_ = ros::Time::now();

    // ROS Setup
    odom_sub_ = nh.subscribe("/odom", 10, &EKFLocalizationWLM::odomCallback, this);
    landmark_sub_ = nh.subscribe("/color_sample", 10, &EKFLocalizationWLM::landmarkCallback, this);
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_pose_wlm", 10);
}

bool EKFLocalizationWLM::loadLandmarks(const std::string& path) {
    try {
        YAML::Node config = YAML::LoadFile(path);
        
        for (const auto& node : config) {
            Landmark lm;
            lm.x = node[0].as<double>();
            lm.y = node[1].as<double>();
            
            if (node[2].IsSequence()) {
                lm.color = node[2][0].as<std::string>();
                lm.id = node[2][1].as<int>();
            } else {
                lm.color = node[2].as<std::string>();
                lm.id = node[3].as<int>();
            }
            
            landmark_map_.push_back(lm);
            ROS_INFO("Loaded landmark: [%.2f, %.2f, %s, %d]", 
                    lm.x, lm.y, lm.color.c_str(), lm.id);
        }
        return true;
    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parsing error: %s", e.what());
        return false;
    }
}

void EKFLocalizationWLM::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    predict(odom);
    publishPose();
}

void EKFLocalizationWLM::landmarkCallback(const landmark_mapper::ColorSample::ConstPtr& msg) {
    // Find matching landmark
    for (const auto& lm : landmark_map_) {
        if (lm.color == msg->color && lm.id == msg->tag_id) {
            updateWithLandmark(msg, lm);
            // publishPose(); // Don't publish here, odom callback will do it
            return;
        }
    }
    ROS_WARN("Landmark [%s, %d] not found in map!", msg->color.c_str(), msg->tag_id);
}

void EKFLocalizationWLM::predict(const nav_msgs::Odometry::ConstPtr& odom) {
    double v = odom->twist.twist.linear.x;
    double w = odom->twist.twist.angular.z;
    double dt = (odom->header.stamp - last_time_).toSec();
    last_time_ = odom->header.stamp;

    double theta = mu_(2);

    // Motion model
    if (std::abs(w) < 1e-5) {
        mu_(0) += v * dt * cos(theta);
        mu_(1) += v * dt * sin(theta);
    } else {
        mu_(0) += (v/w) * (sin(theta + w*dt) - sin(theta));
        mu_(1) += (v/w) * (-cos(theta + w*dt) + cos(theta));
        mu_(2) += w * dt;
    }
    mu_(2) = atan2(sin(mu_(2)), cos(mu_(2)));

    // Jacobian
    Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
    if (std::abs(w) < 1e-5) {
        G(0,2) = -v * dt * sin(theta);
        G(1,2) = v * dt * cos(theta);
    } else {
        G(0,2) = (v/w) * (cos(theta + w*dt) - cos(theta));
        G(1,2) = (v/w) * (sin(theta + w*dt) - sin(theta));
    }

    // Covariance update
    Sigma_ = G * Sigma_ * G.transpose() + R_;
}

void EKFLocalizationWLM::updateWithLandmark(const landmark_mapper::ColorSample::ConstPtr& msg, const Landmark& lm) {
    double dx = lm.x - mu_(0);
    double dy = lm.y - mu_(1);
    double q = dx*dx + dy*dy;
    double sqrt_q = sqrt(q);

    // Expected measurement
    Eigen::Vector2d h;
    h << sqrt_q, atan2(dy, dx) - mu_(2);
    h(1) = atan2(sin(h(1)), cos(h(1)));

    // Measurement Jacobian
    Eigen::Matrix<double, 2, 3> H;
    H << -dx/sqrt_q, -dy/sqrt_q, 0,
          dy/q, -dx/q, -1;

    // Kalman update
    Eigen::Vector2d z;
    z << msg->rho, msg->theta;
    Eigen::Matrix2d S = H * Sigma_ * H.transpose() + Q_landmark_;
    Eigen::Matrix<double, 3, 2> K = Sigma_ * H.transpose() * S.inverse();

    Eigen::Vector2d y = z - h;
    y(1) = atan2(sin(y(1)), cos(y(1)));

    mu_ += K * y;
    mu_(2) = atan2(sin(mu_(2)), cos(mu_(2)));
    Sigma_ = (Eigen::Matrix3d::Identity() - K * H) * Sigma_;
}

void EKFLocalizationWLM::publishPose() {
    geometry_msgs::PoseWithCovarianceStamped out;
    out.header.stamp = last_time_;
    out.header.frame_id = "map";
    out.pose.pose.position.x = mu_(0);
    out.pose.pose.position.y = mu_(1);
    
    tf2::Quaternion q;
    q.setRPY(0, 0, mu_(2));
    out.pose.pose.orientation = tf2::toMsg(q);

    // Fill covariance
    out.pose.covariance[0] = Sigma_(0,0);  // x-x
    out.pose.covariance[1] = Sigma_(0,1);  // x-y
    out.pose.covariance[5] = Sigma_(0,2);  // x-theta
    out.pose.covariance[6] = Sigma_(1,0);  // y-x
    out.pose.covariance[7] = Sigma_(1,1);  // y-y
    out.pose.covariance[11] = Sigma_(1,2); // y-theta
    out.pose.covariance[30] = Sigma_(2,0); // theta-x
    out.pose.covariance[31] = Sigma_(2,1); // theta-y
    out.pose.covariance[35] = Sigma_(2,2); // theta-theta

    pose_pub_.publish(out);

    // Also publish the transform
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = last_time_;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link_ekf";
    transformStamped.transform.translation.x = mu_(0);
    transformStamped.transform.translation.y = mu_(1);
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation = out.pose.pose.orientation;
    tf_broadcaster_.sendTransform(transformStamped);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf_wlm_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    EKFLocalizationWLM ekf(nh, pnh);
    ros::spin();
    return 0;
}