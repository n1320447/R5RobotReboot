#include "DistanceSensor.h"

// SENSOR PROPERTIES
float distance_prev = 0.0;
float prev_time = 0.0;
const size_t ROLLING_AVERAGE_SIZE = 20;
const float SPIKE_MASK = 1.3;
float rolling_average[ROLLING_AVERAGE_SIZE] = {};
size_t arr_i = 0;

DistanceSensor::DistanceSensor(int DATA_PIN) {
    pin = DATA_PIN;
    pinMode(DATA_PIN, INPUT);
}

void DistanceSensor::readSensor() {
    // Get the raw distance from sensor
    int raw_data = analogRead(pin);
    analog_distance = raw_data;
}

float DistanceSensor::mapDataToDist(int analog_value) {
    float distance_cm = 28000.0 / analog_value;

    float cur_time = micros() / 1000000.0;
    float slope = (distance_cm - distance_prev) / (cur_time - prev_time);

    if (distance_cm == INFINITY) {
        return 0.0;
    }

    rolling_average[arr_i] = distance_cm;
    arr_i = (arr_i + 1) % ROLLING_AVERAGE_SIZE;

    float sum = 0.0;
    for (size_t i = 0; i < ROLLING_AVERAGE_SIZE; ++i)
        sum += rolling_average[i];
    // Serial.println(sum / ROLLING_AVERAGE_SIZE, 2);

    distance_prev = distance_cm;
    prev_time = cur_time;

    if (analog_value == 0) {
        // Handle invalid reading
        return -1;
    }
    float voltage = analog_value * (3.3 / 4095.0);
    if (voltage == 0) {
        // Handle invalid voltage
        return -1;
    }

    return distance_cm;
}

float DistanceSensor::getDistance() {
    // Updates, maps, then returns end distance from sensor
    readSensor();
    return mapDataToDist(analog_distance);
}

bool DistanceSensor::objectFound() {
    float distance_cm = getDistance();
    // Check if an object is found based on the distance
    if (distance_cm <= 20) {
        return true;
    }
    return false;
}
