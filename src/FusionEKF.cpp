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
      /**
      Convert radar from polar to Cartesian coordinates and initialize state.
      */

      // TODO: Manually call atan and sqrt instead of doing the Jacobian

      // float px = measurement_pack.raw_measurements_(0);
      // float py = measurement_pack.raw_measurements_(1);
      // float vx = measurement_pack.raw_measurements_(2);
      // float vy = measurement_pack.raw_measurements_(3);
      // if (px == 0 && py == 0) {
      //   cout << "Divide by zero error!" << endl;
      // }
      // float px2 = pow(px, 2);
      // float py2 = pow(py, 2);
      // Hj_ << px / sqrt(px2 + py2), py / sqrt(px2 + py2), 0, 0,
      //     -py / (px2 + py2), px / (px2 + py2), 0, 0,
      //     py*(vx * py - vy * px) / (pow(px2 + py2, 1.5)), px*(vx * py - vy * px) / (pow(px2 + py2, 1.5)), px / sqrt(px2 + py2), py / sqrt(px2 + py2);

      // ekf_.x_ = Hj_ * measurement_pack.raw_measurements_;
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
  float delta_t = measurement_pack.timestamp_ - previous_timestamp_ / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_(0, 2) = delta_t;
  ekf_.F_(1, 3) = delta_t;

  // Update the process noise covariance
  int dt_2 = delta_t * delta_t;
  int dt_3 = dt_2 * delta_t;
  int dt_4 = dt_3 * delta_t;
  int noise_ax = 9; // TODO: Move to a class variable
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
    // ekf_.UpdateEKF(measurement_pack.raw_measurements_); // Radar updates
  } else {
    ekf_.Update(measurement_pack.raw_measurements_); // Laser updates
  }

  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
