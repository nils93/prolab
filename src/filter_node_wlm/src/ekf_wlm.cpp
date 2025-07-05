#include "filter_node_wlm/ekf_wlm.h"
#include <yaml-cpp/yaml.h>
#include <fstream>

EKFLocalizationWLM::EKFLocalizationWLM(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    // Parameter laden
    std::string landmark_path;
    pnh.param<std::string>("landmark_map_path", landmark_path, "");

    if (landmark_path.empty() || !loadLandmarks(landmark_path)) {
        ROS_FATAL("Failed to load landmarks. Shutting down.");
        ros::shutdown();
        return;
    }

    // Initialisierung (ähnlich wie im alten EKF)
    mu_.setZero();
    Sigma_ = Eigen::Matrix3d::Identity() * 0.1;
    R_ = Eigen::Matrix3d::Identity() * 1e-3;
    Q_ = Eigen::Matrix2d::Identity() * 1e-4; // Messunsicherheit verringert
    last_time_ = ros::Time::now();

    // Subscriber
    odom_sub_ = nh.subscribe("/odom", 10, &EKFLocalizationWLM::odomCallback, this);
    landmark_sub_ = nh.subscribe("/color_sample", 10, &EKFLocalizationWLM::landmarkCallback, this);

    // Publisher
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_pose_wlm", 10);
}

void EKFLocalizationWLM::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    predict(odom);
    publishPose(); // Pose nach jedem Prädiktionsschritt publizieren
}

void EKFLocalizationWLM::landmarkCallback(const landmark_mapper::ColorSample::ConstPtr& msg) {
    update(msg);
    publishPose(); // Pose nach jedem Korrekturschritt publizieren
}


bool EKFLocalizationWLM::loadLandmarks(const std::string& path) {
    try {
        YAML::Node landmarks_yaml = YAML::LoadFile(path);
        for (const auto& node : landmarks_yaml) {
            Landmark lm;
            lm.x = node[0].as<double>();
            lm.y = node[1].as<double>();
            lm.color = node[2][0].as<std::string>();
            lm.id = node[2][1].as<int>();
            landmark_map_.push_back(lm);
        }
        ROS_INFO("Loaded %zu landmarks from %s", landmark_map_.size(), path.c_str());
        return true;
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to load or parse landmark file %s: %s", path.c_str(), e.what());
        return false;
    }
}

void EKFLocalizationWLM::predict(const nav_msgs::Odometry::ConstPtr& odom) {
  // Implementierung aus dem alten EKF übernehmen und anpassen
  double v = odom->twist.twist.linear.x;
  double w = odom->twist.twist.angular.z;
  double dt = (odom->header.stamp - last_time_).toSec();
  last_time_ = odom->header.stamp;

  double theta = mu_(2);

  // Bewegungsgleichung (g)
  if (std::abs(w) < 1e-5) {
    mu_(0) += v * dt * cos(theta);
    mu_(1) += v * dt * sin(theta);
  } else {
    mu_(0) += -(v/w) * sin(theta) + (v/w) * sin(theta + w*dt);
    mu_(1) +=  (v/w) * cos(theta) - (v/w) * cos(theta + w*dt);
    mu_(2) += w * dt;
  }
  mu_(2) = atan2(sin(mu_(2)), cos(mu_(2)));

  // Jacobi-Matrix G_t
  Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
  if (std::abs(w) < 1e-5) {
    G(0,2) = -v * dt * sin(theta);
    G(1,2) =  v * dt * cos(theta);
  } else {
    G(0,2) = (v/w) * cos(theta) - (v/w) * cos(theta + w * dt);
    G(1,2) = (v/w) * sin(theta) - (v/w) * sin(theta + w * dt);
  }

  Sigma_ = G * Sigma_ * G.transpose() + R_;
}

void EKFLocalizationWLM::update(const landmark_mapper::ColorSample::ConstPtr& msg) {
    // 1. Datenassoziation: Finde die passende Landmarke in der Karte
    const Landmark* found_lm = nullptr;
    for (const auto& lm : landmark_map_) {
        if (lm.color == msg->color && lm.id == msg->tag_id) {
            found_lm = &lm;
            break;
        }
    }

    if (!found_lm) {
        ROS_WARN("Landmark [\"%s\", %d] not in map. Skipping update.", msg->color.c_str(), msg->tag_id);
        return;
    }

    // 2. Berechne erwartete Messung h(mu) und Jacobi H
    double lm_x = found_lm->x;
    double lm_y = found_lm->y;
    double mu_x = mu_(0);
    double mu_y = mu_(1);
    double mu_theta = mu_(2);

    double dx = lm_x - mu_x;
    double dy = lm_y - mu_y;
    double q = dx * dx + dy * dy;
    double sqrt_q = sqrt(q);

    // Erwartete Messung h(mu)
    Eigen::Vector2d h;
    h(0) = sqrt_q;
    h(1) = atan2(dy, dx) - mu_theta;
    // Winkel normalisieren
    h(1) = atan2(sin(h(1)), cos(h(1)));

    // Mess-Jacobi-Matrix H
    Eigen::Matrix<double, 2, 3> H;
    H << -dx / sqrt_q, -dy / sqrt_q, 0,
          dy / q    , -dx / q     , -1;

    // 3. Kalman-Update
    Eigen::Matrix<double, 2, 2> S = H * Sigma_ * H.transpose() + Q_;
    Eigen::Matrix<double, 3, 2> K = Sigma_ * H.transpose() * S.inverse();

    // Echte Messung z
    Eigen::Vector2d z;
    z << msg->rho, msg->theta;

    // Innovation y
    Eigen::Vector2d y = z - h;
    // Winkeldifferenz normalisieren
    y(1) = atan2(sin(y(1)), cos(y(1)));

    // Update von Zustand und Kovarianz
    mu_ = mu_ + K * y;
    mu_(2) = atan2(sin(mu_(2)), cos(mu_(2))); // Winkel wieder normalisieren
    Sigma_ = (Eigen::Matrix3d::Identity() - K * H) * Sigma_;
    
    ROS_INFO("Updated with landmark [\"%s\", %d]", msg->color.c_str(), msg->tag_id);
}

void EKFLocalizationWLM::publishPose() {
    geometry_msgs::PoseWithCovarianceStamped out;
    out.header.stamp = last_time_; // Timestamp von der letzten Messung verwenden
    out.header.frame_id = "map";
    out.pose.pose.position.x = mu_(0);
    out.pose.pose.position.y = mu_(1);
    tf2::Quaternion q;
    q.setRPY(0, 0, mu_(2));
    out.pose.pose.orientation = tf2::toMsg(q);

    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            out.pose.covariance[i * 6 + j] = (i < 3 && j < 3) ? Sigma_(i, j) : 0;

    pose_pub_.publish(out);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf_wlm_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    EKFLocalizationWLM ekf(nh, pnh);
    ros::spin();
    return 0;
}
