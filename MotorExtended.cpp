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
    Serial.println("Updating Encoder Pos...");
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
    prev_error = (target_position - cumulative_position) + 20;
    proportionalControl(); // Initially call proportional control to start motion towards target
}

void MotorExtended::proportionalControl() {
    Serial.println("Proportional Control...");
    if (!seeking_target) return; // Exit if not seeking target

    Serial.println("Still seeking...");
    long error = target_position - cumulative_position;
    Serial.print("Error is: ");
    Serial.println(error);
    Serial.print("Prev Error is: ");
    Serial.println(prev_error);

    bool getting_better = (error - prev_error) < 0; // If your previous error is greater than your current error, then you are on a good track
    if(getting_better){
      Serial.println("Getting better");
    }
    else{
      Serial.println("Not getting any better...");
    }

    int speed = -1 * static_cast<int>(error * kP); // Calculate proportional speed
    speed = max(-100, min(speed, 100)); // Clamp speed to motor's speed range
    if(abs(speed) <= 50){
      speed = (speed)/(abs(speed))*50; // Sign of the speed * 50 (minimum speed where motor still turns)
    }

    if (abs(error) <= 100 || !getting_better) { // Threshold to stop (error within acceptable range or if error starts getting worse)
        Serial.println("Braking motor...");
        brakeMotor(); // Close enough to target, stop the motor
    } else {
        prev_error = error;
        Serial.print("Still have a ways to go, drive at: ");
        Serial.println(speed);
        analogDriveMotor(speed); // Adjust motor speed based on proportional control
    }
}
