#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in; // current state (pose)
  P_ = P_in; // state covariance matrix
  F_ = F_in; // state transition matrix (holds delta T)
  H_ = H_in; // measurement matrix (maps state space to measurement space)
  R_ = R_in; // measurement covariance matrix
  Q_ = Q_in; // process covariance matrix
}

void KalmanFilter::Predict() {
  x_ = F_ * x_; // update current position using delta T
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_; // update state uncertainty based on delta T and process noise
}

void KalmanFilter::Update(const VectorXd &z) {
  // This is for a LIDAR measurment, no need to linearize

  //measurement covariance matrix - laser
  MatrixXd R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

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
  MatrixXd R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

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
