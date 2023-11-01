#include "RobotMotion.h"
#include "Sensor.h"
#include <iostream>
#include <string>

int main() {
    RobotMotion robot;
    Sensor sensor;
    std::string command;

    while (true) {
        std::cout << "Enter command (forward, backward, left, right, stop, sensor_input, exit): ";
        std::cin >> command;

        if (command == "forward") {
            robot.moveForward();
        } else if (command == "backward") {
            robot.moveBackward();
        } else if (command == "left") {
            robot.turnLeft();
        } else if (command == "right") {
            robot.turnRight();
        } else if (command == "stop") {
            robot.stop();
        } else if (command == "sensor_input") {
            sensor.sensorInput();
        } else if (command == "exit") {
            break;
        } else {
            std::cout << "Invalid command!" << std::endl;
        }
    }

    return 0;
}
