#include "robotintro.h"

RobotInfo::RobotInfo(string name, int speed, int weight, int size, int number_of_sensors) {
  this->name = name;
  this->speed = speed;
  this->weight = weight;
  this->size = size;
  this->number_of_sensors = number_of_sensors;
}

void RobotInfo::IntroduceYourself() {
  cout << "My name is " << this->name << "." << endl;
  cout << "My top speed is " << this->speed << " miles per hour." << endl;
  cout << "My weight is " << this->weight << " kilograms. " << endl;
  cout << "My size is " << this->size << " inches in size" << endl;
}

void RobotInfo::NumOfSensors() {
  cout << "I have altogether " << this->number_of_sensors << " many sensors on my body." << endl;
}
