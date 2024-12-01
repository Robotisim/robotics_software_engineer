#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <string>

class Actuator {
public:
    Actuator(std::string name);
    std::string getName();
protected:
    std::string name;
};

class ArmActuator : public Actuator {
public:
    void move(int angle);
};

class WheelActuator : public Actuator {
public:
    void rotate(int degrees);
};

#endif // ACTUATOR_H
