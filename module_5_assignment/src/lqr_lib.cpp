#include "lqr_lib.hpp"

LQR::LQR(StateMatrix const& Q, InputMatrix const& R, int horizon): Q_(Q), R_(R), horizon_(horizon) {
/*This is the definition of the constructor, where the actual initialization and logic occur. There are a few important points here:

    Member Initializer List (: Q_(Q), R_(R), horizon_(horizon)):
        Q_(Q): This initializes the member variable Q_ (which is presumably declared in the class) with the value of the parameter Q passed to the constructor.
        R_(R): Similarly, R_ is initialized with the value of R.
        horizon_(horizon): The member variable horizon_ is initialized with the horizon parameter.
        */
    A_ = StateMatrix::Zero(3, 3);
    B_ = InputMatrix::Zero(3, 2);
    P_ = Q_; // why we are making P_ and Q_ same

    K_ = StateMatrix::Zero(B_.cols(), A_.rows());

}

LQR::StateMatrix LQR::getA(double yaw, double v, double dt) {

    StateMatrix A(3, 3);
    A << 1, 0, -v * sin(yaw) * dt,
         0, 1, v * cos(yaw) * dt,
         0, 0, 1;
    return A;

}

LQR::InputMatrix LQR::getB(double yaw, double dt) {

    InputMatrix B(3, 2);
    B << cos(yaw) * dt, 0,
         dt * sin(yaw), 0,
         0, dt;
    return B;

}
void LQR::updateMatrices(StateMatrix const& A, InputMatrix const& B) {

    A_ = A;
    B_ = B;

}
void LQR::computeRiccati(InputMatrix B, StateMatrix A) {

    P_ = Q_;
    B_ = B;
    A_ = A;

    for (int i = horizon_; i > 0; --i) {
        Eigen::MatrixXd Y = R_ + B_.transpose() * P_ * B_;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd Yinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();
        P_ = Q_ + A_.transpose() * P_ * A_ - (A_.transpose() * P_ * B_) * Yinv * (B_.transpose() * P_ * A_);
    }

    /* wht we have to use JacobiSVD and svd.matrix and svd.singularValues()?
    - to avoid the numerically unstable matrices
    */

    Eigen::MatrixXd Y = R_ + B_.transpose() * P_ * B_;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd Yinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();
    K_ = Yinv * B_.transpose() * P_ * A_;

}

LQR::InputVector LQR::computeOptimalInput(StateVector const& state_error) {

    InputVector u = -K_ * state_error;
    return u;

}
