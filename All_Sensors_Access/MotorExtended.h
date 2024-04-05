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

    void connectEncoder();
    void updateEncoderPos();
    void analogDriveMotor(int analogSpeed);
    void brakeMotor();
    void encoderDrive(long delta); // Drive the motor based on delta encoder counts

private:
    Motor motor;
    AS5600 encoder;
    long cumulative_position = 0;
    long previous_cumulative_position = 0;
    long offset_cumulative_position = 0;
    long target_position = 0; // Target encoder position to reach
    bool seeking_target = false; // Whether the motor is currently seeking a target position
    int prev_error;

private:
    Motor motor;
    AS5600 encoder;
    long target_position = 0; // Target encoder position to reach
    float kP = 0.1; // Proportional gain for encoderDrive
    int analog_velocity = 0;
    long prev_read_time = 0;

    void proportionalControl(); // Adjusts motor speed based on the proportional control logic
};

#endif
