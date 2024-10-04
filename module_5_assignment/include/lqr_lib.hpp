#ifndef LQR_HPP
#define LQR_HPP

#include <Eigen/Dense>
#include <iostream>

class LQR {
public:
      //dev[x(t)] = Ax + Bu
    //derivate_state = stateTransitionMatrix * stateVector + controlInputMatrix * inputVector


    // Eigen Matrix Initilize
    using StateMatrix = Eigen::MatrixXd;
    using InputMatrix = Eigen::MatrixXd;
    using StateVector = Eigen::VectorXd;
    using InputVector = Eigen::VectorXd;

    // Constructor to initialize LQR with cost matrices Q, R, and prediction horizon
    LQR(StateMatrix const& Q, InputMatrix const& R, int horizon);

    StateMatrix getA(double yaw, double v, double dt);
    InputMatrix getB(double yaw, double dt);
    void updateMatrices(StateMatrix const& A, InputMatrix const& B);

    // Compute the Riccati equation solution to update the gain matrix K
    void computeRiccati(InputMatrix B, StateMatrix A);
    InputVector computeOptimalInput(StateVector const& state_error);
    StateMatrix K_;


private:


    StateMatrix A_;    // representing system dynamics in the state-space model
    InputMatrix B_;    // representing how control inputs affect the state
    StateMatrix Q_;    // to penalize deviations in the state vector
    InputMatrix R_;    // to penalize the use of control inputs
    StateMatrix P_;    // used in computing the optimal state feedback gain matrix (K)
    int horizon_;      // how far into the future the controller optimizes the control actions


};

#endif // LQR_HPP