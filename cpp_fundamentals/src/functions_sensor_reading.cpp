// This program takes an array of sensor readings, calculates their average, and checks against a threshold.
// If the average exceeds a certain threshold, it notifies the user.
// Author: Robotisim

#include <iostream>
using namespace std;

// Calculates the average of an array of sensor readings
int calculateAverage(int readings[5]) {
    int sum = 0;
    for (int i = 0; i < 5; ++i) {
        sum += readings[i];
    }
    return sum / 5;
}

// Checks if the average sensor reading crosses a predefined threshold
void checkThreshold(int average) {
    const int THRESHOLD = 30; // Predefined threshold for sensor readings
    if (average > THRESHOLD) {
        cout << "Sensor Reading is crossing threshold." << endl;
    } else {
        cout << "Sensor Reading is not crossing threshold." << endl;
    }
}

int main() {
    int sensorReadings[5] = {35, 43, 35, 64, 54};
    cout << "Sensor Reading average is " << calculateAverage(sensorReadings) << endl;
    checkThreshold(calculateAverage(sensorReadings));

    int sensorReadings1[5] = {45, 63, 95, 64, 54};
    cout << "Sensor Reading average is " << calculateAverage(sensorReadings1) << endl;
    checkThreshold(calculateAverage(sensorReadings1));

    return 0;
}
