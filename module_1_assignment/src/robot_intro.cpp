#include "robotintro.h"

using namespace std;

int main() {
  // Create a RobotInfo object named robot1
  RobotInfo robot1("R2D2", 300, 70, 130, 25);

  // Call the functions on the created object
  robot1.IntroduceYourself();
  robot1.NumOfSensors();

  return 0;
}
