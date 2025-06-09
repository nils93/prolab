#include "pf.h"

extern ros::Publisher pub_pf_filter;

std::vector<Particle> particles;

std::default_random_engine gen;
std::normal_distribution<double> noise_x(0.0, 0.01);
std::normal_distribution<double> noise_y(0.0, 0.01);
std::normal_distribution<double> noise_theta(0.0, 0.01);

Eigen::VectorXd motionModel(const Eigen::VectorXd& state, const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu) {
  double dt = 0.1;
  double v = odom->twist.twist.linear.x;
  double omega = odom->twist.twist.angular.z;
  double theta = state(2);

  Eigen::VectorXd new_state = state;
  new_state(0) += v * cos(theta) * dt + noise_x(gen);
  new_state(1) += v * sin(theta) * dt + noise_y(gen);
  new_state(2) += omega * dt + noise_theta(gen);

  return new_state;
}

double measurementLikelihood(const Eigen::VectorXd& state, const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu) {
  double dx = state(0) - odom->pose.pose.position.x;
  double dy = state(1) - odom->pose.pose.position.y;
  double distance = sqrt(dx*dx + dy*dy);

  double sigma = 0.1;
  double likelihood = exp(-0.5 * (distance * distance) / (sigma * sigma));
  return likelihood;
}

std::vector<Particle> resampleParticles(const std::vector<Particle>& old_particles) {
  std::vector<Particle> new_particles;
  double total_weight = 0.0;
  for (const auto& p : old_particles) {
    total_weight += p.weight;
  }

  std::vector<double> cumulative;
  double cum_sum = 0.0;
  for (const auto& p : old_particles) {
    cum_sum += p.weight / total_weight;
    cumulative.push_back(cum_sum);
  }

  double step = 1.0 / old_particles.size();
  double r = ((double) rand() / RAND_MAX) * step;
  int idx = 0;

  for (int i = 0; i < old_particles.size(); ++i) {
    double u = r + i * step;
    while (u > cumulative[idx]) {
      idx++;
    }
    new_particles.push_back(old_particles[idx]);
  }

  return new_particles;
}

void pfCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu) {
  for (auto& p : particles) {
    p.state = motionModel(p.state, odom, imu);
  }

  std::vector<double> log_weights;
  for (auto& p : particles) {
    double likelihood = measurementLikelihood(p.state, odom, imu);
    if (likelihood < 1e-300) {
      log_weights.push_back(-1e300);
    } else {
      log_weights.push_back(log(likelihood));
    }
  }

  double max_log_weight = *std::max_element(log_weights.begin(), log_weights.end());
  double sum_exp = 0.0;
  for (const auto& lw : log_weights) {
    sum_exp += exp(lw - max_log_weight);
  }
  double log_total_weight = max_log_weight + log(sum_exp);

  double total_weight = 0.0;
  for (size_t i = 0; i < particles.size(); ++i) {
    particles[i].weight = exp(log_weights[i] - log_total_weight);
    total_weight += particles[i].weight;
  }

  particles = resampleParticles(particles);

  Eigen::VectorXd mean = Eigen::VectorXd::Zero(particles[0].state.size());
  total_weight = 0.0;
  for (const auto& p : particles) {
    mean += p.weight * p.state;
    total_weight += p.weight;
  }

  if (total_weight > 1e-6) {
    mean /= total_weight;
  } else {
    mean.setZero();
  }

  nav_msgs::Odometry filtered_msg = *odom;
  filtered_msg.pose.pose.position.x = mean(0);
  filtered_msg.pose.pose.position.y = mean(1);
  pub_pf_filter.publish(filtered_msg);
}
