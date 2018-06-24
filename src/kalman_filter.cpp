#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout; using std::endl;

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

}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  x_ = F_ * x_; // update current position using delta T
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_; // update state uncertainty based on delta T and process noise
}

void KalmanFilter::Update(const VectorXd &z) {
  // This is for a LIDAR measurment, no need to linearize

  // measurement covariance matrix - laser
  VectorXd y = z - H_laser_ * x_; // 2x1 = 2x1 - 2x4 * 4x1
  MatrixXd Ht = H_laser_.transpose(); // 4x2 = 2x4.T
  MatrixXd S = H_laser_ * P_ * Ht + R_laser_; // 2x2 = 2x4 * 4x4 * 4x2 + 2x2
  MatrixXd Si = S.inverse(); // 2x2 = 2x2.inv()
  MatrixXd PHt = P_ * Ht; // 4x2 = 4x4 * 4x2
  MatrixXd K = PHt * Si; // 4x2 = 4x2 * 2x2

  x_ = x_ + (K * y); // 4x1 = 4x1 + 4x2 * 2x1
  long x_size = x_.size(); // 4
  MatrixXd I = MatrixXd::Identity(x_size, x_size); // 4x4
  P_ = (I - K * H_laser_) * P_; // 4x4 = (4x4 - 4x2 * 2x4) * 4x4

  // cout << "***" << endl;
  // cout << (I - K * H_laser_) << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state using Extended Kalman Filter equations
  */

  // measurement covariance matrix - radar
  VectorXd y = z - H_radar_ * x_;
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
