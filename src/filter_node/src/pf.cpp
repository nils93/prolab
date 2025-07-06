#include "pf_node/pf_localization.h"

PFLocalization::PFLocalization(ros::NodeHandle& nh)
: odom_sub_(nh, "/odom", 1),
  imu_sub_(nh,  "/imu",  1),
  sync_(SyncPolicy(10), odom_sub_, imu_sub_)
{
  // ### Initiale Pose aus TF holen (Ground Truth) ###
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped t;
  double x_init = 0.0, y_init = 0.0, yaw_init = 0.0;

  ros::Duration(1.0).sleep(); // Warte kurz auf TF
  try {
    t = tfBuffer.lookupTransform("map", "base_link",
                                 ros::Time(0), ros::Duration(1.0));
    x_init = t.transform.translation.x;
    y_init = t.transform.translation.y;
    yaw_init = tf2::getYaw(t.transform.rotation);
    last_time_ = t.header.stamp;
    ROS_INFO("PF initial pose set from TF: x=%.2f, y=%.2f, yaw=%.2f", x_init, y_init, yaw_init);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("PF initial TF lookup failed: %s. Defaulting to (0,0,0).", ex.what());
    last_time_ = ros::Time::now();
  }

  // Initiale Partikel um die Startpose herum verteilen
  particles_.resize(num_particles_);
  std::normal_distribution<double> dist_x(x_init, 0.01); // Kleine Unsicherheit
  std::normal_distribution<double> dist_y(y_init, 0.01);
  std::normal_distribution<double> dist_theta(yaw_init, 0.01);

  for (auto& p : particles_) {
    p.x = dist_x(rng_);
    p.y = dist_y(rng_);
    p.theta = dist_theta(rng_);
    p.weight = 1.0 / num_particles_;
  }

  // Rauschparameter
  alpha1_ = 0.1;
  alpha2_ = 0.1;
  alpha3_ = 0.1;
  alpha4_ = 0.1;

  sync_.registerCallback(
    boost::bind(&PFLocalization::pfCallback, this, _1, _2)
  );

  pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pf_pose", 10);
  particle_pub_ = nh.advertise<geometry_msgs::PoseArray>("pf_particles", 10);
  laser_sub_ = nh.subscribe("/scan", 1, &PFLocalization::laserCallback, this);
  map_sub_   = nh.subscribe("/map",  1, &PFLocalization::mapCallback, this);
}

void PFLocalization::pfCallback(
  const nav_msgs::Odometry::ConstPtr& odom,
  const sensor_msgs::Imu::ConstPtr& imu)
{
  double v = odom->twist.twist.linear.x;
  double w = odom->twist.twist.angular.z;
  double dt = (odom->header.stamp - last_time_).toSec();
  last_time_ = odom->header.stamp;

  motionUpdate(v, w, dt);
  publishPose();
  publishParticles();
}

void PFLocalization::motionUpdate(double v, double w, double dt) {
  std::normal_distribution<double> dist1(0, std::sqrt(alpha1_ * v * v + alpha2_ * w * w));
  std::normal_distribution<double> dist2(0, std::sqrt(alpha3_ * v * v + alpha4_ * w * w));
  std::normal_distribution<double> dist3(0, std::sqrt(alpha1_ * v * v + alpha2_ * w * w));

  for (auto& p : particles_) {
    double v_hat = v + dist1(rng_);
    double w_hat = w + dist2(rng_);
    double gamma_hat = dist3(rng_);

    if (std::abs(w_hat) > 1e-6) {
      double r = v_hat / w_hat;
      p.x += -r * sin(p.theta) + r * sin(p.theta + w_hat * dt);
      p.y +=  r * cos(p.theta) - r * cos(p.theta + w_hat * dt);
    } else {
      p.x += v_hat * dt * cos(p.theta);
      p.y += v_hat * dt * sin(p.theta);
    }

    p.theta += w_hat * dt + gamma_hat;
    p.theta = atan2(sin(p.theta), cos(p.theta));  // Normierung
  }
}

void PFLocalization::publishPose() {
  geometry_msgs::PoseWithCovarianceStamped out;
  out.header.stamp = ros::Time::now();
  out.header.frame_id = "map";

  // Pose = gewichteter Mittelwert
  double x = 0, y = 0, theta = 0;
  for (const auto& p : particles_) {
    x     += p.weight * p.x;
    y     += p.weight * p.y;
    theta += p.weight * p.theta;
  }

  out.pose.pose.position.x = x;
  out.pose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  out.pose.pose.orientation = tf2::toMsg(q);

  // Kovarianz schätzen (3x3 aus [x, y, theta])
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto& p : particles_) {
    Eigen::Vector3d diff(p.x - x, p.y - y, angles::shortest_angular_distance(theta, p.theta));
    cov += p.weight * diff * diff.transpose();
  }

  // In 6x6 eintragen
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      out.pose.covariance[i * 6 + j] = cov(i, j);

  pose_pub_.publish(out);
}

void PFLocalization::publishParticles() {
  geometry_msgs::PoseArray array;
  array.header.stamp = ros::Time::now();
  array.header.frame_id = "map";

  for (const auto& p : particles_) {
    geometry_msgs::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, p.theta);
    pose.orientation = tf2::toMsg(q);
    array.poses.push_back(pose);
  }

  particle_pub_.publish(array);
}

void PFLocalization::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  map_ = *msg;
  map_received_ = true;
  ROS_INFO_ONCE("Map empfangen.");
}

void PFLocalization::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  if (!map_received_) return;
  measurementUpdate(scan);
  resample();
}

void PFLocalization::measurementUpdate(const sensor_msgs::LaserScan::ConstPtr& scan) {
  const int step = 10; // Scanstrahlen nur jede 10. auswerten
  const double z_hit = 0.8;
  const double z_rand = 0.2;
  const double sigma = 0.2;

  for (auto& p : particles_) {
    double weight = 1e-6;

    for (size_t i = 0; i < scan->ranges.size(); i += step) {
      double angle = p.theta + scan->angle_min + i * scan->angle_increment;
      double dist  = scan->ranges[i];
      if (dist < scan->range_min || dist > scan->range_max) continue;

      double mx = p.x + dist * cos(angle);
      double my = p.y + dist * sin(angle);

      int map_x = (mx - map_.info.origin.position.x) / map_.info.resolution;
      int map_y = (my - map_.info.origin.position.y) / map_.info.resolution;
      int index = map_y * map_.info.width + map_x;

      if (map_x >= 0 && map_x < (int)map_.info.width &&
          map_y >= 0 && map_y < (int)map_.info.height) {
        int8_t cell = map_.data[index];
        if (cell > 50) {
          weight += z_hit;
        } else {
          weight += z_rand;
        }
      }
    }

    p.weight = weight;
  }

  // Normalisieren
  double sum = 0.0;
  for (const auto& p : particles_) sum += p.weight;
  for (auto& p : particles_) p.weight /= (sum + 1e-6);
}

void PFLocalization::resample() {
  // Sortiere Partikel nach Gewicht (absteigend)
  std::sort(particles_.begin(), particles_.end(),
            [](const Particle& a, const Particle& b) { return a.weight > b.weight; });

  // Behalte die besten 50%
  int keep = num_particles_ / 2;
  std::vector<Particle> new_particles(particles_.begin(), particles_.begin() + keep);

  // Neue verrauschte Partikel aus den besten 50%
  std::normal_distribution<double> noise_x(0, 0.05);
  std::normal_distribution<double> noise_y(0, 0.05);
  std::normal_distribution<double> noise_theta(0, 0.05);

  for (int i = 0; i < num_particles_ - keep; ++i) {
    const Particle& base = new_particles[i % keep];
    Particle p;
    p.x = base.x + noise_x(rng_);
    p.y = base.y + noise_y(rng_);
    p.theta = base.theta + noise_theta(rng_);
    p.theta = atan2(sin(p.theta), cos(p.theta));
    p.weight = 1.0 / num_particles_;
    new_particles.push_back(p);
  }

  particles_ = new_particles;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pf_node");
  // Erzeuge NodeHandle im privaten Namensraum (~)
  ros::NodeHandle nh("~");
  PFLocalization pf(nh);
  ros::spin();
  return 0;
}