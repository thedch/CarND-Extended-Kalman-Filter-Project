#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
  public:

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement covariance matrix
    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;

    // measurement matrix
    Eigen::MatrixXd H_laser_;
    Eigen::MatrixXd H_radar_;
    // H is the matrix that projects your belief about the object's current state
    // into the measurement space of the sensor. For lidar, this is a fancy way of
    // saying that we discard velocity information from the state variable since
    // the lidar sensor only measures position

    KalmanFilter();

    virtual ~KalmanFilter();

    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     */
    void Predict();

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    void Update(const Eigen::VectorXd &z);

    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     */
    void UpdateEKF(const Eigen::VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */
