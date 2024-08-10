<h1> Kalman Filter </h1>

The Kalman filter is an algorithm that uses a series of measurements observed over time, containing statistical noise and other inaccuracies, and produces estimates of unknown variables that tend to be more accurate than those based on a single measurement alone. More formally, the Kalman filter operates recursively on streams of noisy input data to produce a statistically optimal estimate of the underlying system state.

<h3> Kalman Filter Equations </h3>

The Kalman filter is based on linear dynamic systems discretized in time. The state of the system is described by the following equations:

1. State prediction:
    - x<sub>k</sub> = A * x<sub>k-1</sub> + B * u<sub>k</sub> + w<sub>k</sub>
    - P<sub>k</sub> = A * P<sub>k-1</sub> * A<sup>T</sup> + Q
    - where:
        - x<sub>k</sub> is the state vector at time k
        - A is the state transition matrix
        - B is the control input matrix
        - u<sub>k</sub> is the control vector at time k
        - w<sub>k</sub> is the process noise at time k
        - P<sub>k</sub> is the state covariance matrix at time k
        - Q is the process noise covariance matrix

2. Measurement update:
   - K<sub>k</sub> = P<sub>k</sub> * H<sup>T</sup> * (H * P<sub>k</sub> * H<sup>T</sup> + R)<sup>-1</sup>
   - x<sub>k</sub> = x<sub>k</sub> + K<sub>k</sub> * (z<sub>k</sub> - H * x<sub>k</sub>)
   - P<sub>k</sub> = (I - K<sub>k</sub> * H) * P<sub>k</sub>
   - where:
        - K<sub>k</sub> is the Kalman gain at time k
        - H is the observation matrix
        - R is the measurement noise covariance matrix
        - z<sub>k</sub> is the measurement vector at time k
        - I is the identity matrix
        - x<sub>k</sub> is the state vector at time k
        - P<sub>k</sub> is the state covariance matrix at time k

<h3> Kalman Filter Algorithm </h3>

The Kalman filter algorithm consists of two main steps: prediction and update. The prediction step uses the state transition matrix A and the control input matrix B to predict the state of the system at the next time step. The update step uses the measurement matrix H to correct the predicted state based on the observed measurements.

<h2> Our Problem </h2>

We have a differential drive robot that moves in a 2D plane. The robot has two wheels, and we can control the linear and angular velocities of each wheel independently. We have access to the curent state vector that contains the robot's position (x, y) and orientation (theta). We also have access to the control vector that contains the linear and angular velocities of the robot's wheels. Our goal is to estimate the robot's state using the Kalman filter.

<h3> State Vector </h3>

The state vector x contains the robot's position (x, y) and orientation (theta). The state vector is defined as follows:

x = [x, y, theta]<sup>T</sup>

where:
- x is the x-coordinate of the robot's position
- y is the y-coordinate of the robot's position
- theta is the orientation of the robot
- T denotes the transpose operation

<h3> Control Vector </h3>

The control vector u contains the linear and angular velocities of the robot's wheels. The control vector is defined as follows:

u = [v, w]<sup>T</sup>

where:
- v is the linear velocity of the robot
- w is the angular velocity of the robot
- T denotes the transpose operation

<h3> Observation Vector </h3>

The observation vector z contains the measurements of the robot's position (x, y) and orientation (theta). The observation vector is defined as follows:

z = [x, y, theta]<sup>T</sup>

where:
- x is the x-coordinate of the robot's position
- y is the y-coordinate of the robot's position
- theta is the orientation of the robot
- T denotes the transpose operation

<h3> State Transition Matrix </h3>

The state transition matrix A relates the state vector x at time k to the state vector x at time k-1. The state transition matrix is defined as follows:

A = [1, 0, 0]
    [0, 1, 0]
    [0, 0, 1]

<h3> Control Input Matrix </h3>

The control input matrix B relates the control vector u to the state vector x. The control input matrix is defined as follows:

B = [cos(theta), 0]
    [sin(theta), 0]
    [0, 1]

<h3> Observation Matrix </h3>

The observation matrix H relates the state vector x to the observation vector z. The observation matrix is defined as follows:

H = [1, 0, 0]
    [0, 1, 0]
    [0, 0, 1]

<h3> Process Noise Covariance Matrix </h3>

The process noise covariance matrix Q represents the uncertainty in the process model. The process noise covariance matrix is defined as follows:

Q = [0.1, 0, 0]
    [0, 0.1, 0]
    [0, 0, 0.1]

<h3> Measurement Noise Covariance Matrix </h3>

The measurement noise covariance matrix R represents the uncertainty in the measurements. The measurement noise covariance matrix is defined as follows:

R = [0.1, 0, 0]
    [0, 0.1, 0]
    [0, 0, 0.1]

<h3> Initial State </h3>

The initial state vector x<sub>0</sub> and the initial state covariance matrix P<sub>0</sub> are given as follows:

x<sub>0</sub> = [0, 0, 0]<sup>T</sup>

P<sub>0</sub> = [1, 0, 0]
            [0, 1, 0]
            [0, 0, 1]

<h3> Implementation </h3>

We will implement the Kalman filter algorithm to estimate the robot's state using the provided state transition matrix A, control input matrix B, observation matrix H, process noise covariance matrix Q, measurement noise covariance matrix R, initial state vector x<sub>0</sub>, and initial state covariance matrix P<sub>0</sub>. We will use the control vector u and observation vector z to update the state estimate and state covariance matrix at each time step.  

Let's implement the Kalman filter algorithm in C++.
```cpp
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

int main() {
    // Define the state transition matrix A
    Matrix3d A;
    A << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    // Define the control input matrix B
    Matrix<double, 3, 2> B;
    B << cos(theta), 0,
         sin(theta), 0,
         0, 1;

    // Define the observation matrix H
    Matrix3d H = Matrix3d::Identity();

    // Define the process noise covariance matrix Q
    Matrix3d Q;
    Q << 0.1, 0, 0,
         0, 0.1, 0,
         0, 0, 0.1;

    // Define the measurement noise covariance matrix R
    Matrix3d R;
    R << 0.1, 0, 0,
         0, 0.1, 0,
         0, 0, 0.1;

    // Define the initial state vector x0
    Vector3d x = Vector3d::Zero();

    // Define the initial state covariance matrix P0
    Matrix3d P = Matrix3d::Identity();

    // Define the control vector u
    Vector2d u;
    u << v, w;

    // Define the observation vector z
    Vector3d z;
    z << x, y, theta;

    // Kalman filter algorithm
    for (int i = 0; i < num_steps; i++) {
        // Prediction step
        x = A * x + B * u;
        P = A * P * A.transpose() + Q;

        // Update step
        Matrix3d K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        x = x + K * (z - H * x);
        P = (Matrix3d::Identity() - K * H) * P;

        // Print the estimated state vector
        std::cout << "Estimated state vector x: " << x.transpose() << std::endl;
    }

    return 0;
}
```


