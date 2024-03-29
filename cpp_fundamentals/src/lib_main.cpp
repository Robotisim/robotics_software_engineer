// Demonstrates integrating Eigen for matrix operations, OpenCV for image processing,
// and Boost.Asio for asynchronous operations in a robotics context.
// Utilizes a custom LineFollowingRobot class for managing a robot's line-following sensors and decision-making.
// Author: Robotisim

#include "lib_line_following.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <vector>

using namespace std;

int main() {
    // Demonstrating a matrix operation using Eigen
    Eigen::MatrixXd mat(2, 2);
    mat(0, 0) = 3;
    mat(1, 1) = 3;
    cout << "Matrix operation result: " << mat.inverse() << endl;

    // Placeholder for image processing with OpenCV
    cv::Mat image;
    cout << "Applying image processing on sensor data..." << endl;

    // Setting up Boost.Asio for asynchronous operations
    boost::asio::io_context io;
    cout << "Starting asynchronous sensor data read..." << endl;

    // Initializing the line-following robot with specific sensor pins and executing its functions
    vector<int> sensorPins = {2, 3, 4};
    LineFollowingRobot robot(3, sensorPins);
    robot.calibrateSensors();
    robot.readSensors();
    robot.processSensorData();
    robot.decideAction();
    return 0;
}
