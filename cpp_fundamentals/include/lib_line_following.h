#ifndef LINEFOLLOWINGROBOT_H
#define LINEFOLLOWINGROBOT_H

#include <vector>
#include <iostream>
using namespace std;

class LineFollowingRobot {
    private:
        int numberOfSensors;
        vector<int> sensorPins;

    public:
        LineFollowingRobot(int numSensors, const vector<int> &pins);
        void readSensors();
        void calibrateSensors();
        void processSensorData();
        void decideAction();
};
#endif // LINEFOLLOWINGROBOT_H