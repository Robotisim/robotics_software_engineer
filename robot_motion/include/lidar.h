#ifndef LIDAR_H

#define LIDAR_H
#include "sensor.h"

class lidar : public Sensor{
    public :
    void sensor_output() override;
};


#endif