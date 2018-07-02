#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

  if (!is_initialized_) {
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 5, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      int rho = measurement_pack.raw_measurements_(0);
      int theta = measurement_pack.raw_measurements_(1);
      ekf_.x_(0) = rho * cos(theta);
      ekf_.x_(1) = rho * sin(theta);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return; // done initializing, no need to predict or update, just return
  }

  // *** Prediction Step ***

  // Determine the time elapsed and update the state transition matrix accordingly
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.Update_F_with_dt(dt);
  ekf_.Update_Q_with_dt(dt);

  ekf_.Predict();

  // ***  Update Step ***

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.H_radar_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_); // Radar updates
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.Update(measurement_pack.raw_measurements_); // Laser updates
  }

  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
