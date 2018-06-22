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

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
           0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
           0, 0.0009, 0,
           0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  ekf_.F_ = MatrixXd(4, 4); // Initialize the state transition matrix
  ekf_.F_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;
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
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to Cartesian coordinates and initialize state.
      */
      float px = measurement_pack.raw_measurements_(0);
      float py = measurement_pack.raw_measurements_(1);
      float vx = measurement_pack.raw_measurements_(2);
      float vy = measurement_pack.raw_measurements_(3);
      if (px == 0 && py == 0) {
        cout << "Divide by zero error!" << endl;
      }
      float px2 = pow(px, 2);
      float py2 = pow(py, 2);
      Hj_ << px / sqrt(px2 + py2), py / sqrt(px2 + py2), 0, 0,
          -py / (px2 + py2), px / (px2 + py2), 0, 0,
          py*(vx * py - vy * px) / (pow(px2 + py2, 1.5)), px*(vx * py - vy * px) / (pow(px2 + py2, 1.5)), px / sqrt(px2 + py2), py / sqrt(px2 + py2);

      // Use Hj.T to convert radar readings into cartesian space
      // Set ekf_.x_ = the current position

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      // Set ekf_.x_ = the current position
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  long delta_t = measurement_pack.timestamp_ - previous_timestamp_ / 1000000.0;
  ekf_.F_[0][2] = delta_t;
  ekf_.F_[1][3] = delta_t;

  dt_2 = delta_t * delta_t;
  dt_3 = dt_2 * delta_t;
  dt_4 = dt_3 * delta_t;
  int noise_ax = 9;
  int noise_ay = 9;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
          0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
          dt_3 / 2 * noise_ax, 0, dt_2 / 1 * noise_ax, 0,
          0, dt_3 / 2 * noise_ay, 0, dt_2 / 1 * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  // Measurement Update:
  // y = z - H*x (this is the error)
  // S = H*P*H.T + R (error is mapped into matrix S which is obtained by projecting the system uncertainty into the measurement space)
  // K = P*H.T*S.inv (kalman gain)
  // x' = x + K*y (update estimate)
  // P' = (I - K*H) * P (update uncertainty)
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
