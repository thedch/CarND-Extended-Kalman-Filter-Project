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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to Cartesian coordinates.
    */

    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 0, 0;

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
  float dt = measurement_pack.timestamp_ - previous_timestamp_ / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_(0, 2) = dt; // TODO: Move to kalman filter class method
  ekf_.F_(1, 3) = dt;

  // Update the process noise covariance
  // TODO: Move to kalman filter class method
  int dt_2 = dt * dt;
  int dt_3 = dt_2 * dt;
  int dt_4 = dt_3 * dt;
  int noise_ax = 9;
  int noise_ay = 9;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
          0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
          dt_3 / 2 * noise_ax, 0, dt_2 / 1 * noise_ax, 0,
          0, dt_3 / 2 * noise_ay, 0, dt_2 / 1 * noise_ay;

  ekf_.Predict();

  // ***  Update Step ***

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.H_radar_ = tools.CalculateJacobian(ekf_.x_);
    // ekf_.UpdateEKF(measurement_pack.raw_measurements_); // Radar updates
  } else {
    ekf_.Update(measurement_pack.raw_measurements_); // Laser updates
  }

  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
