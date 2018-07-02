#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout; using std::endl;

VectorXd map_pose_into_radar(VectorXd);
double normalize_angle(double angle);

KalmanFilter::KalmanFilter() {
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  H_radar_ = MatrixXd(3, 4);

  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0, // Project the current state -> LIDAR measurement space
              0, 1, 0, 0;

  P_ = MatrixXd(4, 4); // Initialize the state covariance matrix
  P_ << 1, 1, 1, 1,
        1, 1, 1, 1,
        1, 1, 1, 1,
        1, 1, 1, 1;

  F_ = MatrixXd(4, 4); // Initialize the state transition matrix
  F_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

  Q_ = MatrixXd(4, 4);

}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  x_ = F_ * x_; // update current position using delta T
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_; // update state uncertainty based on delta T and process noise
}

void KalmanFilter::Update(const VectorXd &z) {
  // measurement covariance matrix - laser
  VectorXd y = z - H_laser_ * x_; // 2x1 = 2x1 - 2x4 * 4x1
  MatrixXd Ht = H_laser_.transpose(); // 4x2 = 2x4.T
  MatrixXd S = H_laser_ * P_ * Ht + R_laser_; // 2x2 = 2x4 * 4x4 * 4x2 + 2x2
  MatrixXd Si = S.inverse(); // 2x2 = 2x2.inv()
  MatrixXd PHt = P_ * Ht; // 4x2 = 4x4 * 4x2
  MatrixXd K = PHt * Si; // 4x2 = 4x2 * 2x2

  x_ = x_ + (K * y); // 4x1 = 4x1 + (4x2 * 2x1)
  long x_size = x_.size(); // 4
  MatrixXd I = MatrixXd::Identity(x_size, x_size); // 4x4
  P_ = (I - K * H_laser_) * P_; // 4x4 = (4x4 - 4x2 * 2x4) * 4x4

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  // measurement covariance matrix - radar
  VectorXd y = z - map_pose_into_radar(x_);
  y(1) = normalize_angle(y(1));

  MatrixXd Ht = H_radar_.transpose();
  MatrixXd S = H_radar_ * P_ * Ht + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y); // Update state using measurement
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_radar_) * P_; // Update uncertainty covariance
}

// Maps the current pose into radar state space to facilitate the error calculation
VectorXd map_pose_into_radar(VectorXd x_) {
  VectorXd h = VectorXd(3); // polar: rho, phi, rho prime
  h << 0,0,0;

  double sqrt_sum = sqrt(x_(0)*x_(0) + x_(1)*x_(1)); // sqrt of x^2 + y^2
  sqrt_sum = std::max(sqrt_sum, 0.00001);

  h(0) = sqrt_sum;
  h(1) = atan2(x_(1), x_(0));
  h(2) = (x_(0)*x_(2) + x_(1)*x_(3)) / sqrt_sum;
  return h;
}

// Adds or subtracts in multiples of 2PI until angle is between PI and -PI
double normalize_angle(double angle) {
  while (angle > M_PI) {
    angle -= 2*M_PI;
  }
  while (angle < -M_PI) {
    angle += 2*M_PI;
  }
  return angle;
}

void KalmanFilter::Update_F_with_dt(float dt) {
  F_(0, 2) = dt; // TODO: Move to kalman filter class method
  F_(1, 3) = dt;
}

void KalmanFilter::Update_Q_with_dt(float dt) {
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  int noise_ax = 9;
  int noise_ay = 9;

  Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
             0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
             dt_3 / 2 * noise_ax, 0, dt_2 / 1 * noise_ax, 0,
             0, dt_3 / 2 * noise_ay, 0, dt_2 / 1 * noise_ay;
}
