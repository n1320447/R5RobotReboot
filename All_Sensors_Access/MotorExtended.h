#ifndef MOTOREXTENDED_H
#define MOTOREXTENDED_H

#include <Wire.h>
#include <AS5600.h>
#include <TB6612_ESP32.h>

class MotorExtended {
public:
    // Constructor declarations
    MotorExtended(int IN1, int IN2, int PWM, int STBY_PIN, int CHANNEL, TwoWire& wire, int DIRECTION = 1);
    MotorExtended(int IN1, int IN2, int PWM, int STBY_PIN, int CHANNEL, int DIRECTION = 1);

    // Other member function declarations...
    void connectEncoder();
    void updateEncoderPos();
    void analogDriveMotor(int analogSpeed);
    void brakeMotor();
    
private:
    // Member variables...
    long cumulative_position;
    long previous_cumulative_position;
    float angular_velocity;
    float previous_angular_velocity;
    int analog_velocity;
    int prev_analog_velocity;
    long offset_cumulative_position;
    float radians_per_tick;
    long prev_read_time;
    float linear_velocity;
    // const float diameter;
    Motor motor;
    AS5600 encoder;
};

#endif
