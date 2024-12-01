#include "Actuator.h"
#include <iostream>

Actuator::Actuator(std::string name) : name(name) {}

std::string Actuator::getName() {
    return name;
}

void ArmActuator::move(int angle) {
    std::cout << "Moving arm actuator " << name << " to angle " << angle << "." << std::endl;
}

void WheelActuator::rotate(int degrees) {
    if (degrees > 360) {
        std::cout << "Rotation angle is too large!" << std::endl;
    }
    std::cout << "Rotating wheel actuator " << name << " by " << degrees << " degrees." << std::endl;
}
