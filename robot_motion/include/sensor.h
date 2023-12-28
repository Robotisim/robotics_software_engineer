#ifndef SENSOR_H
#define SENSOR_H

#include <iostream>

class Sensor {
public:
    virtual void sensor_output() = 0;
    virtual ~Sensor() {}
};

#endif // SENSOR_H
