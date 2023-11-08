// Run this using gcc main.cpp -lstdc++ -o robot_motion
// ./robot_motion

#include <iostream>
#include <string>

void moveForward() {
    std::cout << "Moving forward" << std::endl;
}

void turnLeft() {
    std::cout << "Turning left" << std::endl;
}

void turnRight() {
    std::cout << "Turning right" << std::endl;
}

int main() {
    std::string command;
    while (true) {
        std::cout << "Enter command (forward, left, right, exit): ";
        std::cin >> command;

        if (command == "forward") {
            moveForward();
        } else if (command == "left") {
            turnLeft();
        } else if (command == "right") {
            turnRight();
        } else if (command == "exit") {
            break;
        } else {
            std::cout << "Invalid command!" << std::endl;
        }
    }
    return 0;
}
