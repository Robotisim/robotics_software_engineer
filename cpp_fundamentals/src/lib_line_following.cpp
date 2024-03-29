// This class implements a line-following robot using a set number of sensors.
// It handles sensor readings, calibration, and processes sensor data to make decisions.
// Author: Robotisim

#include "lib_line_following.h"


LineFollowingRobot::LineFollowingRobot(int numSensors, const vector<int> &pins)
    : numberOfSensors(numSensors), sensorPins(pins) {
  cout << "Robot created with " << numberOfSensors << " sensors." << endl;
}
void LineFollowingRobot::readSensors() {
  cout << "Reading line sensors..." << endl;
  for (int pin : sensorPins) {
    cout << "Reading sensor at pin " << pin << endl;
  }
}
void LineFollowingRobot::calibrateSensors() {
  cout << "Calibrating line sensors..." << endl;
}
void LineFollowingRobot::processSensorData() {
  cout << "Applying PID to Sensor values.." << endl;
}

void LineFollowingRobot::decideAction() {
  cout << "Deciding action based on sensor data..." << endl;
}
