#include "pf_node/pf_localization.h"

PFLocalization::PFLocalization(ros::NodeHandle& nh) {
    // Landmarken laden
    XmlRpc::XmlRpcValue lm_list;
    nh.getParam("landmarks", lm_list);
    for (int i = 0; i < lm_list.size(); ++i) {
        Landmark L;
        L.id = (int)lm_list[i]["id"];
        L.x  = (double)lm_list[i]["x"];
        L.y  = (double)lm_list[i]["y"];
        LM_.push_back(L);
    }

    // Partikel initialisieren
    initParticles(1000);  // Anzahl anpassbar
    last_time_ = ros::Time::now();

    // ROS-Subs/Pubs
    odom_sub_ = nh.subscribe("/odom", 10, &PFLocalization::odomCallback, this);
    lm_sub_   = nh.subscribe("landmark_obs", 10, &PFLocalization::lmCallback, this);
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pf_pose", 10);
    particle_cloud_pub_ = nh.advertise<geometry_msgs::PoseArray>("particle_cloud", 10);  // Neu
}

void PFLocalization::initParticles(int N) {
    particles_.resize(N);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);  // Initiale Verteilung anpassen
    for (auto& p : particles_) {
        p.pose << dist(rng_), dist(rng_), dist(rng_) * M_PI;
        p.weight = 1.0 / N;
    }
}

void PFLocalization::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    predict(msg);
    publishPose();
}

void PFLocalization::lmCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    int id = msg->header.frame_id.empty() ? 0 : std::stoi(msg->header.frame_id);
    Eigen::Vector2d z(msg->pose.pose.position.x, msg->pose.pose.position.y);
    update(id, z);
}

void PFLocalization::predict(const nav_msgs::Odometry::ConstPtr& odom) {
    ros::Time now = odom->header.stamp;
    double dt = (now - last_time_).toSec();
    last_time_ = now;

    double v = odom->twist.twist.linear.x;
    double w = odom->twist.twist.angular.z;
    std::normal_distribution<double> noise(0.0, 0.05);  // Rauschen anpassen

    for (auto& p : particles_) {
        double theta = p.pose(2);
        p.pose(0) += (v * dt * cos(theta)) + noise(rng_);
        p.pose(1) += (v * dt * sin(theta)) + noise(rng_);
        p.pose(2) += (w * dt) + noise(rng_);
    }
}

void PFLocalization::update(int id, const Eigen::Vector2d& z) {
    // Landmarke finden
    auto it = std::find_if(LM_.begin(), LM_.end(), [&](const Landmark& L){ return L.id == id; });
    if (it == LM_.end()) return;

    // Gewichte aktualisieren (Likelihood)
    double sum_weights = 0.0;
    for (auto& p : particles_) {
        double dx = it->x - p.pose(0);
        double dy = it->y - p.pose(1);
        double r_pred = sqrt(dx*dx + dy*dy);
        double b_pred = atan2(dy, dx) - p.pose(2);
        
        // Gaußsche Likelihood (Rauschen anpassen)
        double likelihood = exp(-0.5 * (pow(z(0) - r_pred, 2) / 0.1 + pow(z(1) - b_pred, 2) / 0.1));
        p.weight *= likelihood;
        sum_weights += p.weight;
    }

    // Normalisieren
    for (auto& p : particles_) p.weight /= sum_weights;

    // Resampling
    resample();
}

void PFLocalization::resample() {
    std::vector<Particle> new_particles;
    std::vector<double> weights;
    for (const auto& p : particles_) weights.push_back(p.weight);

    std::discrete_distribution<int> dist(weights.begin(), weights.end());
    for (size_t i = 0; i < particles_.size(); ++i) {
        int idx = dist(rng_);
        new_particles.push_back(particles_[idx]);
        new_particles.back().weight = 1.0 / particles_.size();  // Gewichte zurücksetzen
    }
    particles_ = std::move(new_particles);
}

void PFLocalization::publishPose() {
    // ROS-Nachricht erstellen
    geometry_msgs::PoseWithCovarianceStamped out;
    out.header.stamp    = ros::Time::now();
    out.header.frame_id = "map";

    // Pose als Mittelwert der Partikel
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& p : particles_) mean += p.pose;
    mean /= particles_.size();
    
    out.pose.pose.position.x = mean(0);
    out.pose.pose.position.y = mean(1);
    tf2::Quaternion q; 
    q.setRPY(0, 0, mean(2));
    out.pose.pose.orientation = tf2::toMsg(q);

    // Empirische Covariance aus Partikeln
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : particles_) {
        Eigen::Vector3d diff = p.pose - mean;
        cov += diff * diff.transpose();
    }
    cov /= particles_.size();

    // 6×6-Covariance (nur Position/Orientation)
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            out.pose.covariance[i*6 + j] = (i < 3 && j < 3) ? cov(i, j) : 0;
        }
    }

    // Partikelwolke als PoseArray publizieren
    geometry_msgs::PoseArray particle_cloud;
    particle_cloud.header.stamp = ros::Time::now();
    particle_cloud.header.frame_id = "map";

    for (const auto& p : particles_) {
        geometry_msgs::Pose pose;
        pose.position.x = p.pose(0);
        pose.position.y = p.pose(1);
        tf2::Quaternion q;
        q.setRPY(0, 0, p.pose(2));
        pose.orientation = tf2::toMsg(q);
        particle_cloud.poses.push_back(pose);
    }

    particle_cloud_pub_.publish(particle_cloud);  // Partikelwolke senden

    pose_pub_.publish(out);
}