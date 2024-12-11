#ifndef EKF_LIB_HPP
#define EKF_LIB_HPP

#include <Eigen/Dense>

// IMU Readings: [omega, a]
// GPS Readings: [x_gps, y_gps]
// State Vector: [x, y, vx, vy, theta]
// Measurements: [GPS_readings, IMU_readings]

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();

    void initialize(Eigen::VectorXd const& x_in,
                    Eigen::MatrixXd const& P_in,
                    Eigen::MatrixXd const& F_in,
                    Eigen::MatrixXd const& H_in,
                    Eigen::MatrixXd const& R_in,
                    Eigen::MatrixXd const& Q_in);

    // Prediction Step
    void predict();

    // Measurement Prediction
    void measurement_prediction();

    // Update Step
    void update(Eigen::VectorXd const& z);

    // Matrices Updating
    void updateF();
    void updateR(std::vector<double> const& R_in);

    Eigen::VectorXd x_;
    Eigen::VectorXd x_pred_;
    Eigen::VectorXd z_pred_;
    double dt;

private:


    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd Q_;
    double epsilon = 1e-6;
    void normalizeAngle(double& angle);
    void update_state();
};

#endif