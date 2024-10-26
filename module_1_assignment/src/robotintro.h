#ifndef INTROROBOT_H
#define INTROROBOT_H

// include libraries
#include <vector>
#include <iostream>
#include <string>

using namespace std;

// creating a class
class RobotInfo {
private:
  string name;
  int speed;
  int weight;
  int size;
  int number_of_sensors;

public:
  // Constructor declaration
  RobotInfo::RobotInfo(string name, int speed, int weight, int size, int number_of_sensors);

  // Function declarations (no return type)
  void IntroduceYourself();
  void NumOfSensors();
};

#endif // INTROROBOT_H
