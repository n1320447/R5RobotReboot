#include "Robot.h"
#include <Arduino.h>

Robot::Robot(int ain1, int ain2, int pwma, int stby, int bin1, int bin2, int pwmb, int dist_pin, TwoWire& i2c)
  : motor1(ain1, ain2, pwma, stby, 1, 1),
    motor2(bin1, bin2, pwmb, stby, 2, i2c, 1),
    dist1(dist_pin),
    i2c(i2c),
    occupancyGrid(),
    pathPlanner(occupancyGrid){}

void Robot::updateAll() {
  Serial.println("Trying to update sensors");
  motor1.updateEncoderPos();
  motor2.updateEncoderPos();
}

void Robot::moveForward(int speed) {
  motor1.analogDriveMotor(speed * 1.00);
  motor2.analogDriveMotor(speed);
}

void Robot::moveReverse() {
  motor1.analogDriveMotor(50);
  motor2.analogDriveMotor(50);
  delay(1000); // Wait for 1 second

  motor1.brakeMotor(); // If there's a brake function
  motor2.brakeMotor(); // If there's a brake function
}

void Robot::reverseFromLeft() {
  motor1.analogDriveMotor(100);
  motor2.analogDriveMotor(0);
  delay(1000); // Wait for 1 second

  motor1.brakeMotor(); // If there's a brake function
  motor2.brakeMotor(); // If there's a brake function
}

void Robot::reverseFromRight() {
  motor1.analogDriveMotor(0);
  motor2.analogDriveMotor(100);
  delay(1000);
  motor1.brakeMotor(); // If there's a brake function
  motor2.brakeMotor(); // If there's a brake function
}

void Robot::turnLeft() {
  motor1.analogDriveMotor(-100);
  motor2.analogDriveMotor(0);
  delay(800); // Wait for 1 second

  motor1.brakeMotor(); // If there's a brake function
  motor2.brakeMotor(); // If there's a brake function
}

void Robot::turnRight() {
  motor1.analogDriveMotor(0);
  motor2.analogDriveMotor(-100);
  delay(1000); // Wait for 1 second
  // Stop the motors after 1 second
  motor1.brakeMotor(); // If there's a brake function
  motor2.brakeMotor(); // If there's a brake function
}

void Robot::scanLeftandRight() {
  int found = 0;
  turnLeft();
  reverseFromLeft();
  turnRight();
  reverseFromRight();
}

void Robot::brakeMotors() {
  motor1.brakeMotor(); // If there's a brake function
  motor2.brakeMotor(); // If there's a brake function
}

void Robot::getClearOfObject() {
  float distance1 = dist1.getDistance();
  int left = 0;
  int right = 0;
  while (distance1 <= 10) {
    turnRight();
    distance1 = dist1.getDistance();
    if (distance1 <= 10) right = 1;
    if (right == 0) {
      turnRight();
      break;
    }
    reverseFromRight();
    turnLeft();
    distance1 = dist1.getDistance();
    if (distance1 <= 10) left = 1;
    if (left == 0) {
      turnLeft();
      break;
    }
    reverseFromLeft();
    if (left == 0 && right == 0) {
      moveForward(-100);
      break;
    }
    distance1 = dist1.getDistance();
  }
}


// void Robot::turnInCircle() {
//   // Assuming motor1 is on the left and motor2 is on the right for this example
//   // Adjust motor power and duration based on your specific robot's design and desired circle size
//   motor1.analogDriveMotor(100); // Drive left motor forward
//   motor2.analogDriveMotor(-100); // Drive right motor in reverse
//   delay(1000); // Duration of turn, adjust this to complete a circle

//   // Optionally, stop the motors after completing the turn
//   brakeMotors();
// }



void Robot::seedRound() {
  turnRight();
  moveForward(-150);
  delay(250);
  turnLeft();
  delay(250);
  Serial.println("Start moving forward...");
  int objectsDetected = 0;
  while (true) {
    Serial.println(dist1.getDistance());// Check distance...
    moveForward(-155); //Move forward no matter what? SHouldn't this only happen if object not found?
    if (dist1.objectFound()) {
      Serial.println("found object dist1");
      getClearOfObject();
      objectsDetected++;
      Serial.print("objectDetect:");
    }
  }
  Serial.println("moving left...");
  moveReverse();
  turnLeft();
  moveForward(-150);
  delay(1000);
  Serial.println("turning right...");
  turnRight();
  moveForward(-150);
  delay(1000);
  Serial.println("break motors...");
  brakeMotors();
}
