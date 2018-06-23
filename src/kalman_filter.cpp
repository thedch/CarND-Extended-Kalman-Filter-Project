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

  H_ = MatrixXd(2, 4); // Project current state -> LIDAR measurement space
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;

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
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred; // TODO: y = z - H*x
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y); // Update state using measurement
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_; // Update uncertainty covariance
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state using Extended Kalman Filter equations
  */

  // measurement covariance matrix - laser
  MatrixXd Hj(3, 4);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  if (px == 0 && py == 0) {
    std::cout << "Divide by zero error in Update EKF!" << std::endl;
    return;
  }

  float px2 = pow(px, 2);
  float py2 = pow(py, 2);
  Hj << px / sqrt(px2 + py2), py / sqrt(px2 + py2), 0, 0,
        -py / (px2 + py2), px / (px2 + py2), 0, 0,
        py*(vx * py - vy * px) / (pow(px2 + py2, 1.5)), px*(vx * py - vy * px) / (pow(px2 + py2, 1.5)), px / sqrt(px2 + py2), py / sqrt(px2 + py2);

  // measurement covariance matrix - radar
  VectorXd z_pred = Hj * x_;
  VectorXd y = z - z_pred; // TODO: y = z - H*x
  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj * P_ * Ht + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y); // Update state using measurement
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_; // Update uncertainty covariance

}
