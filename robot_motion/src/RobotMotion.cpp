#include "RobotMotion.h"
#include <iostream>

void RobotMotion::moveForward() {
    std::cout << "Robot is moving forward." << std::endl;
}

void RobotMotion::moveBackward() {
    std::cout << "Robot is moving backward." << std::endl;
}

void RobotMotion::turnLeft() {
    std::cout << "Robot is turning left." << std::endl;
}

void RobotMotion::turnRight() {
    std::cout << "Robot is turning right." << std::endl;
}

void RobotMotion::stop() {
    std::cout << "Robot has stopped." << std::endl;
}
