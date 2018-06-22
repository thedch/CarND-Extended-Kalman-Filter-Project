#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4); // TODO: Can I hardcode these values?
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
        cout << "Error in CalculateRMSE" << endl;
        return rmse;
    }

    // accumulate squared residuals
    for (int i = 0; i < estimations.size(); ++i) {
        VectorXd diff = estimations[i] - ground_truth[i];
        diff = diff.array() * diff.array();
        rmse += diff;
    }

    // calculate the mean
    rmse = rmse / estimations.size();

    // calculate the squared root
    rmse = rmse.array().sqrt();

    // return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3, 4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // check division by zero
    if (px == 0 && py == 0) {
        cout << "Divide by zero error!" << endl;
        return Hj;
    }

    // compute the Jacobian matrix
    float px2 = pow(px, 2);
    float py2 = pow(py, 2);
    Hj << px / sqrt(px2 + py2), py / sqrt(px2 + py2), 0, 0,
    -py / (px2 + py2), px / (px2 + py2), 0, 0,
    py*(vx * py - vy * px) / (pow(px2 + py2, 1.5)), px*(vx * py - vy * px) / (pow(px2 + py2, 1.5)), px / sqrt(px2 + py2), py / sqrt(px2 + py2);

    return Hj;
}
