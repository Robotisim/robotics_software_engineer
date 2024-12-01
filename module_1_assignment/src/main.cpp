#include "Robot.h"
#include "Actuator.h"

int main() {
    // Instantiate robot and actuators
    Robot myRobot("Explorer", 5.0);
    ArmActuator arm("Left Arm");
    WheelActuator wheel("Front Left");

    // Simulate actuator operations
    myRobot.activateActuator(arm);
    arm.move(90);

    wheel.rotate(-30);

    myRobot.deactivateActuator(arm);

    return 0;
}