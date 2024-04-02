#include "MotorExtended.h"

// Constructor for non-default I2C bus
MotorExtended::MotorExtended(int IN1, int IN2, int PWM, int STBY_PIN, int CHANNEL, TwoWire& wire, int DIRECTION)
    : motor(IN1, IN2, PWM, DIRECTION, STBY_PIN, 5000, 8, CHANNEL), 
      encoder(&wire) {
    // Initialization using the specified I2C bus
}

// Constructor for using the default I2C bus
MotorExtended::MotorExtended(int IN1, int IN2, int PWM, int STBY_PIN, int CHANNEL, int DIRECTION)
    : motor(IN1, IN2, PWM, DIRECTION, STBY_PIN, 5000, 8, CHANNEL), 
      encoder(&Wire) {
    // Initialization using the default I2C bus
}

void MotorExtended::connectEncoder() {
    // Try to connect to motor
    while (!encoder.isConnected()) {
        Serial.println("Trying to connect...");
        encoder.begin();
        delay(1000);
    }

    Serial.println("Connected!");

    // Find and set offset for encoders
    offset_cumulative_position = encoder.getCumulativePosition();
}

void MotorExtended::updateEncoderPos() {
    // Updates the encoder position attribute

    // Updates previous position
    if (previous_cumulative_position) {
        previous_cumulative_position = cumulative_position;
    } else {
        previous_cumulative_position = 0;
    }

    // Updates time
    prev_read_time = millis();

    // Gets new position
    cumulative_position = encoder.getCumulativePosition() - offset_cumulative_position;
}

void MotorExtended::analogDriveMotor(int analogSpeed) {
    // Does basic drive for motor by providing value between -255, 255
    // TODO: implement an amount of time to drive for without blocking
    prev_analog_velocity = analog_velocity;
    analog_velocity = analogSpeed;

    motor.drive(analog_velocity);
}

void MotorExtended::brakeMotor() {
    // Brakes the motor
    motor.brake();
}
