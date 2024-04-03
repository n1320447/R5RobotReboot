#include "MotorExtended.h"

// Constructor for non-default I2C bus
MotorExtended::MotorExtended(int IN1, int IN2, int PWM, int STBY_PIN, int CHANNEL, TwoWire& wire, int DIRECTION)
    : motor(IN1, IN2, PWM, DIRECTION, STBY_PIN, 5000, 8, CHANNEL), encoder(&wire) {
    // Initialization using the specified I2C bus
}

// Constructor for using the default I2C bus
MotorExtended::MotorExtended(int IN1, int IN2, int PWM, int STBY_PIN, int CHANNEL, int DIRECTION)
    : motor(IN1, IN2, PWM, DIRECTION, STBY_PIN, 5000, 8, CHANNEL), encoder(&Wire) {
    // Initialization using the default I2C bus
}

void MotorExtended::connectEncoder() {
    // Try to connect to motor encoder
    while (!encoder.isConnected()) {
        Serial.println("Trying to connect...");
        encoder.begin();
        delay(1000);
    }
    Serial.println("Connected!");
    offset_cumulative_position = encoder.getCumulativePosition();
}

void MotorExtended::updateEncoderPos() {
    previous_cumulative_position = cumulative_position;
    cumulative_position = encoder.getCumulativePosition() - offset_cumulative_position;
    proportionalControl(); // Call proportional control to adjust speed as necessary
}

void MotorExtended::analogDriveMotor(int analogSpeed) {
    analog_velocity = analogSpeed;
    motor.drive(analog_velocity);
}

void MotorExtended::brakeMotor() {
    motor.brake();
    seeking_target = false; // Reset seeking target upon manual brake
}

void MotorExtended::encoderDrive(long delta) {
    updateEncoderPos(); // Ensure the current position is up-to-date
    target_position = cumulative_position + delta;
    seeking_target = true;
    proportionalControl(); // Initially call proportional control to start motion towards target
}

void MotorExtended::proportionalControl() {
    if (!seeking_target) return; // Exit if not seeking target

    long error = target_position - cumulative_position;
    int speed = static_cast<int>(error * kP); // Calculate proportional speed
    speed = max(-255, min(speed, 255)); // Clamp speed to motor's speed range

    if (abs(error) < 10) { // Threshold to stop, adjust based on your requirements
        brakeMotor(); // Close enough to target, stop the motor
    } else {
        analogDriveMotor(speed); // Adjust motor speed based on proportional control
    }
}
