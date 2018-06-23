#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  P_ << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;

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
  VectorXd y = z - H_laser_ * x_;
  MatrixXd Ht = H_laser_.transpose();
  MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y); // Update state using measurement
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_laser_) * P_; // Update uncertainty covariance
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state using Extended Kalman Filter equations
  */

  // measurement covariance matrix - radar
  VectorXd z_pred = H_radar_ * x_;
  VectorXd y = z - z_pred; // TODO: y = z - H*x
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
