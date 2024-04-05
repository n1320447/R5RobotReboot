#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include <Arduino.h>

class DistanceSensor {
public:
    DistanceSensor(int DATA_PIN);
    float getDistance();
    bool objectFound();

private:
    int pin;
    int analog_distance;

    void readSensor();
    float mapDataToDist(int analog_value);
};

#endif
