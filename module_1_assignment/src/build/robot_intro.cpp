#include "robotintro.h"

using namespace std;

class RobotInfo {
private:
    string name;
    int speed; // miles per hour (assuming)
    int weight; // kilograms (assuming)
    int size; // inches (assuming)
    int number_of_sensors;

public:
    RobotInfo(string name, int speed, int weight, int size, int number_of_sensors) {
        this->name = name;
        this->speed = speed;
        this->weight = weight;
        this->size = size;
        this->number_of_sensors = number_of_sensors;
    }

    void IntroduceYourself() {
        cout << "My name is " << name << "." << endl;
        cout << "My top speed is " << speed << " miles per hour." << endl;
        cout << "My weight is " << weight << " kilograms. " << endl;
        cout << "My size is " << size << " inches in size." << endl;
    }

    void NumOfSensors() {
        cout << "I have altogether " << number_of_sensors << " sensors on my body." << endl;
    }
};

int main() {
    // Create a RobotInfo object named robot1
    RobotInfo robot1("R2D2", 300, 70, 130, 25);

    // Call the functions on the created object
    robot1.IntroduceYourself();
    robot1.NumOfSensors();

    return 0;
}
