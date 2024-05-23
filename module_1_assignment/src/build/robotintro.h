// Guards
#ifdef INTROROBOT_H
#define INTROROBOT_H

// include libraries
#include <vector>
#include <iostream>
using namespace std;

// creating a class

class RobotInfo
{
private:
    string name;
    int speed;
    int weight;
    int size;
    int number_of_sensors;

public:
    RobotInfo(string name, int speed, int weight, int size, int number_of_sensors) void IntroduceYourself();
    void NumofSensors();
};
#endif