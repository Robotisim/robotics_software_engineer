#include "ekf_lib.hpp"
#include <iostream>
#include <math.h>

// IMU Readings: [omega, a]
// GPS Readings: [x_gps, y_gps]
// State Vector: [x, y, vx, vy, theta]
// Measurements: [GPS_readings, IMU_readings]
// The object being measured is a car

ExtendedKalmanFilter::ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::initialize(Eigen::VectorXd const& x_in,
                                      Eigen::MatrixXd const& P_in,
                                      Eigen::MatrixXd const& F_in,
                                      Eigen::MatrixXd const& H_in,
                                      Eigen::MatrixXd const& R_in,
                                      Eigen::MatrixXd const& Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void ExtendedKalmanFilter::predict() {
    std::cout << "Predicting the next state based on the current state and time step" << std::endl;
    update_state();
    updateF();
    P_ = F_ * P_ * F_.transpose() + Q_;
    std::cout << "Predicted covariance matrix (P) updated." << std::endl;
}

void ExtendedKalmanFilter::measurement_prediction() {
    double dx = x_pred_[0] - x_[0];
	double dy = x_pred_[1] - x_[1];
    double exp_x = x_pred_[0];
    double exp_y = x_pred_[1];
    double exp_omega = (x_pred_[4] - x_[4]) / dt;
    double exp_a = (sqrt(pow(x_pred_[2] - x_[2], 2)) + (pow(x_pred_[3] - x_[3], 2))) / dt;
    z_pred_ << exp_x, exp_y, exp_a, exp_omega;
    std::cout << "Predicted measurement vector z_pred_: " << z_pred_.transpose() << std::endl;

    double a_d_Vx = (1 / (exp_a + epsilon)) * (x_pred_[2] * dx) / (dt * sqrt(pow(dx, 2) + pow(dy, 2)) + epsilon);
    double a_d_Vy = (1 / (exp_a + epsilon)) * (x_pred_[3] * dy) / (dt * sqrt(pow(dx, 2) + pow(dy, 2)) + epsilon);

    std::cout << "Intermediate calculation a_d_Vx: " << a_d_Vx << ", a_d_Vy: " << a_d_Vy << std::endl;

    H_(0, 0) = 1;
    H_(1, 1) = 1;
    H_(2, 2) = a_d_Vx;
    H_(2, 3) = a_d_Vy;
    H_(3, 4) = 1;
    // std::cout << "Measurement matrix H updated: " << std::endl << H_ << std::endl;
}

void ExtendedKalmanFilter::update(Eigen::VectorXd const& z) {
    measurement_prediction();
    // Measurement prediction completed. Predicted measurement vector (z_pred_):
    std::cout << "------ Actual measurement vector (z): " << z.transpose() << std::endl;
    std::cout << "------ Prediction vector (z_pred_): " << z_pred_.transpose() << std::endl;

    Eigen::VectorXd y = z - z_pred_;
    std::cout << "Measurement residual (y = z - z_pred_): " << y.transpose() << std::endl;

    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    // std::cout << "Residual covariance (S = H * P * H^T + R): " << S << std::endl;

    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    // std::cout << "Kalman gain (K = P * H^T * S^-1): " << K << std::endl;

    x_ = x_ + K * y;
    std::cout << "Updated state vector (x = x + K * y): " << x_.transpose() << std::endl;

    P_ = (Eigen::MatrixXd::Identity(5, 5) - K * H_) * P_;
    // std::cout << "Updated covariance matrix (P = (I - K * H) * P): " << P_ << std::endl;

    normalizeAngle(x_[4]);
}

void ExtendedKalmanFilter::updateF() {
    F_.diagonal().setOnes();
    F_(0, 2) = dt;
    F_(1, 3) = dt;
    F_(4, 4) = 1;
}

void ExtendedKalmanFilter::updateR(std::vector<double> const& R_in) {
    R_.diagonal() = Eigen::VectorXd::Map(R_in.data(), R_in.size());
    // std::cout << "Measurement noise covariance matrix R updated: " << R_.diagonal().transpose() << std::endl;
}

void ExtendedKalmanFilter::normalizeAngle(double& angle) {
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
}

void ExtendedKalmanFilter::update_state() {
    // Motion Model
    x_pred_[0] = x_[0] + x_[2] * dt; // new_position_x = current_position_x + velocity_x * time_step
    x_pred_[1] = x_[1] + x_[3] * dt; // new_position_y = current_position_y + velocity_y * time_step
    x_pred_[2] = x_[2];
    x_pred_[3] = x_[3];
    x_pred_[4] = x_[4];
    normalizeAngle(x_pred_[4]);
    std::cout << "State prediction completed" << std::endl << "Predicted state vector x_pred: " << x_pred_.transpose() << std::endl;
}