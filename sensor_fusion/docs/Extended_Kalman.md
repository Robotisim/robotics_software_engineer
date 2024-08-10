<h1> Extended Kalman Filter </h1>

<h3> Introduction </h3>

The Kalman filter is a linear algorithm that estimates the state of a linear dynamic system from a series of noisy measurements. The Extended Kalman Filter (EKF) is a non-linear version of the Kalman filter that can handle non-linear systems. The EKF linearizes the system at the current mean and covariance and then applies the Kalman filter to the linearized system. The EKF is an iterative algorithm that can be used to estimate the state of a non-linear system.

<h3> Algorithm </h3>

The EKF algorithm consists of two main steps: the prediction step and the update step. The prediction step uses the system dynamics to predict the state of the system at the next time step. The update step uses the measurements to correct the predicted state.

The prediction step consists of the following equations:

1. Predict the state of the system:
    x_hat = f(x, u)

2. Predict the covariance of the state:
        P_hat = F * P * F^T + Q
        
        Where:
        - x_hat is the predicted state of the system
        - f is the system dynamics function
        - x is the current state of the system
        - u is the control input
        - P is the covariance of the state
        - F is the Jacobian of the system dynamics function
        - Q is the process noise covariance
        - T is the transpose operator

The update step consists of the following equations:

1. Compute the Kalman gain:
        K = P_hat * H^T * (H * P_hat * H^T + R)^-1
        
        Where:
        - K is the Kalman gain
        - H is the Jacobian of the measurement function
        - R is the measurement noise covariance

        Update the state of the system:
        x = x_hat + K * (z - h(x_hat))
        
        Where:
        - x is the updated state of the system
        - z is the measurement
        - h is the measurement function
        - K is the Kalman gain
        - x_hat is the predicted state of the system

        Update the covariance of the state:
        P = (I - K * H) * P_hat
        
        Where:
        - P is the updated covariance of the state
        - I is the identity matrix
        - K is the Kalman gain
        - H is the Jacobian of the measurement function

<h3> Our Problem </h3>

In our problem we have data from the Kitti dataset. We will mainly be making use of the GPS and IMU data to estimate the position of the car (x,y,theta). The GPS data is noisy and the IMU data is non-linear. We will use the EKF to estimate the position of the vehicle. My measurement vector will contain x and y coordinates from the GPS, it would also contain the velocity in the x and y directions and the angular velocity around the z axis from the imu.

<h4> State Vector </h4>

The state vector will contain the following variables:
- x: x coordinate of the car
- y: y coordinate of the car
- theta: orientation of the car

<h4> Measurement Vector </h4>

The measurement vector will contain the following variables:
- x_gps: x coordinate of the car from the GPS
- y_gps: y coordinate of the car from the GPS
- vx_imu: velocity in the x direction from the IMU
- vy_imu: velocity in the y direction from the IMU
- w_imu: angular velocity around the z axis from the IMU

<h4> System Dynamics Functions </h4>

The system dynamics function will be the following:
- x_hat = x + vx_imu * dt
- y_hat = y + vy_imu * dt
- theta_hat = theta + w_imu * dt

<h4> Measurement Function </h4>

The measurement function will be the following:
- x_gps = x
- y_gps = y
- vx_imu = vx_imu
- vy_imu = vy_imu
- w_imu = w_imu
- z = [x_gps, y_gps, vx_imu, vy_imu, w_imu]




