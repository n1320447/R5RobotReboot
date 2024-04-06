#include "Robot.h"
#include <Arduino.h>

Robot::Robot(int ain1, int ain2, int pwma, int stby, int bin1, int bin2, int pwmb, int dist_pin, TwoWire& i2c)
: motor1(ain1, ain2, pwma, stby, 1, i2c, 1),
  motor2(bin1, bin2, pwmb, stby, 2, i2c, 1),
  dist1(dist_pin),
  i2c(i2c),
  occupancyGrid(),
  pathPlanner(occupancyGrid){}

void Robot::updateAll() {
  Serial.println("Updating Motor1...");
  motor1.updateEncoderPos();
  Serial.println("Updating Motor2...");
  motor2.updateEncoderPos();
  if (isTraveling) {
    long motor1Pos = motor1.cumulative_position;
    long motor2Pos = motor2.cumulative_position;
    long error = motor1Pos - motor2Pos;

    // Correction is a simplified proportional control (adjust factor as needed)
    int correction = error / 10; 

    motor1.analogDriveMotor(int(100 - correction)); 
    motor2.analogDriveMotor(int(100 + correction)); 

    // If either motor reaches the target, stop both (could adjust for stricter conditions)
    if (motor1Pos >= targetTicks || motor2Pos >= targetTicks) {
      brakeMotors();
      isTraveling = false;
    }
  }
}

void Robot::linearDrive(int ticks) {
  motor1.connectEncoder(); // Assuming encoders need to be connected
  motor2.connectEncoder();
  motor1.updateEncoderPos(); // Get current position to set accurate target
  motor2.updateEncoderPos();
  targetTicks = motor1.cumulative_position + ticks; // Set target relative to current position
  motor1.analogDriveMotor(100); // Start motors at initial speed
  motor2.analogDriveMotor(100);
  isTraveling = true;
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
  isTraveling = false; // Ensure we stop traveling when braking
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

long Robot::distToEncoderTicks(float distance){
  float wheel_diam = 2.6;
  float wheel_circ = 3.14159 * wheel_diam;
  float dist_per_step = wheel_circ/4096;

  long encoder_steps = long(distance / dist_per_step);

  Serial.print("For distance: ");
  Serial.println(distance);
  Serial.print("Yields this many encoder steps: ");
  Serial.println(encoder_steps);

  return encoder_steps;
}

void Robot::turn90Degrees(int direction){
  //+1 for right turn, -1 for left turn
  //Drives each wheel a quarter of a circumfrence of the whole bot

  long quarter_turn_ticks = distToEncoderTicks(3.93); //mystery number is (pi*bot_diam)/4 = bot_circum/4

  long left_dist = quarter_turn_ticks  * direction;
  long right_dist = quarter_turn_ticks * -1 * direction;

  Serial.print("left_dist is: ");
  Serial.println(left_dist);
  Serial.print("right_dist is: ");
  Serial.println(right_dist);

  motor1.encoderDrive(left_dist);
  motor2.encoderDrive(right_dist);

  while(motor1.seeking_target || motor2.seeking_target){
    Serial.println("Still moving...");
    updateAll();
  }
  Serial.println("Done!");

}

void Robot::turn45Degrees(int direction){
  //+1 for right turn, -1 for left turn
  //Drives each wheel a quarter of a circumfrence of the whole bot

  long quarter_turn_ticks = distToEncoderTicks(3.93/2); //mystery number is (pi*bot_diam)/4 = bot_circum/4

  long left_dist = quarter_turn_ticks  * direction;
  long right_dist = quarter_turn_ticks * -1 * direction;

  Serial.print("left_dist is: ");
  Serial.println(left_dist);
  Serial.print("right_dist is: ");
  Serial.println(right_dist);

  motor1.encoderDrive(left_dist);
  motor2.encoderDrive(right_dist);

  while(motor1.seeking_target || motor2.seeking_target){
    Serial.println("Still moving...");
    updateAll();
  }
  Serial.println("Done!");

}

void Robot::forwardOneSquare(){
  //Attemptign forward Travel of square distance

  long encoder_ticks_distance = distToEncoderTicks(square_len);

  //First leg
  Serial.println("Performing forward square travel...");
  motor1.encoderDrive(encoder_ticks_distance);
  motor2.encoderDrive(encoder_ticks_distance);
  while(motor1.seeking_target || motor2.seeking_target){
    Serial.println("Still moving...");
    updateAll();
  }
  Serial.println("Finished forward square travel");
  delay(500);
}


void Robot::leftOneSquare(){

  //Turn right
  Serial.println("Performing turn left...");
  turn90Degrees(-1);
  Serial.println("Finished First turn");
  delay(500);

  forwardOneSquare();
}

void Robot::rightOneSquare(){
  //Turn right
  Serial.println("Performing turn right...");
  turn90Degrees(1);
  Serial.println("Finished First turn");
  delay(500);

  forwardOneSquare();
}

void Robot::seedRound() {
  turnRight();
  moveForward(-150);
  delay(250);
  turnLeft();
  delay(250);
  Serial.println("Start moving forward...");
  int objectsDetected = 0;
  while (true) {
    Serial.println(dist1.getDistance());
    moveForward(-155);
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

void Robot::testRound(){
    delay(10000);
    // float wheel_diam = 2.6; // Inches
    // float dist_to_travel_inches = 12;
    
    // // Calculate the circumference of the wheel
    // float wheel_circumference = 3.14159 * wheel_diam;
    
    // // Calculate distance per encoder step
    // float distance_per_step = wheel_circumference / 4096;
    
    // // Calculate how many steps are needed to travel the desired distance
    // int encoder_steps = int(dist_to_travel_inches / distance_per_step);
    float dist_to_travel_inches = 24;
    long encoder_steps = distToEncoderTicks(dist_to_travel_inches);

    Serial.print("For the distance: ");
    Serial.print(dist_to_travel_inches);
    Serial.println(" inches");
    Serial.print("I need to travel: ");
    Serial.print(encoder_steps);
    Serial.println(" encoder steps");

    motor1.encoderDrive(encoder_steps);
    motor2.encoderDrive(encoder_steps);
    Serial.println("Trying to encoder Drive...");

  while(motor1.seeking_target || motor2.seeking_target){
    Serial.println("Still seeking...");
    updateAll(); 
  }
  // motor1.analogDriveMotor(255);
  // motor2.analogDriveMotor(-255);
  delay(1000);
  Serial.println("We ran!");
  brakeMotors();
}

void Robot::driveInSquare(){
  delay(10000);
  rightOneSquare();
  delay(500);
  rightOneSquare();
  delay(500);
  rightOneSquare();
  delay(500);
  rightOneSquare();
  delay(500);
  brakeMotors();
  Serial.println("Done with square");
}

bool Robot::scanFront(){
  Serial.println("inside scanFront method");
  float distance1 = (dist1.getDistance()); /// this would have to be sensor distance1 = (dist1.getDistance() * 2.54) / 4 since distance
  //sensor reading is in cm and each 'square' is 4 inches x 4 inches.
  //check if something is less than or equal to max_distance
  Serial.println(distance1);
  if(distance1 <= max_distance){
    Serial.print("RobotCurrentPositionX: ");
    Serial.println(RobotCurrentPositionX);
    Serial.print("RobotCurrentPositionY: ");
    Serial.println(RobotCurrentPositionY);

    Serial.println("inside scanFront if");

    
    if(distance1 <= 6){
      //2.54 cm in 1 inch
      int squares = distance1/4;
      for(int i = 1; i <= squares; i++){
        Serial.println("ycoordinate is: " + RobotCurrentPositionY-squares);
        occupancyGrid.markCell(RobotCurrentPositionX-i,(RobotCurrentPositionY),CLEAR);
      }
      occupancyGrid.markCell(RobotCurrentPositionX-squares-1,(RobotCurrentPositionY),OCCUPIED);


      return true;
      
    }
    else if(distance1 <= 10.0){
      //found object at 10" or less
      int squares = distance1/4;
      for(int i = 1; i <= squares; i++){
        Serial.println("ycoordinate is: " + RobotCurrentPositionY-squares);
        occupancyGrid.markCell(RobotCurrentPositionX-i,(RobotCurrentPositionY),CLEAR);
      }
      occupancyGrid.markCell(RobotCurrentPositionX-squares-1,(RobotCurrentPositionY),OCCUPIED);
      return true;
    } 
    else if (distance1 <= 14.0){
      //14
      int squares = distance1/4;
      for(int i = 1; i <= squares; i++){
        Serial.println("ycoordinate is: " + RobotCurrentPositionY-squares);
        occupancyGrid.markCell(RobotCurrentPositionX-i,(RobotCurrentPositionY),CLEAR);
      }
      occupancyGrid.markCell(RobotCurrentPositionX-squares-1,(RobotCurrentPositionY),OCCUPIED);
      return true;
    }
  }

  return false;

}

void Robot::turnSlightlyLeft() {
    // This method turns the robot slightly to the left
    motor1.analogDriveMotor(-50); // Negative speed for reverse
    motor2.analogDriveMotor(50);  // Positive speed for forward
    delay(100); // Adjust this delay to control the turn amount

    brakeMotors(); // Stop the motors after the turn
}

void Robot::turnSlightlyRight() {
    // This method turns the robot slightly to the right
    motor1.analogDriveMotor(50);  // Positive speed for forward
    motor2.analogDriveMotor(-50); // Negative speed for reverse
    delay(100); // Adjust this delay to control the turn amount

    brakeMotors(); // Stop the motors after the turn
}

bool Robot::scanDiagonals() {
    Serial.println("Scanning diagonals");

    // Turn slightly left to check the left diagonal
    // turnSlightlyLeft();
    // Assume getDistance reads the distance after the slight left turn
    // float leftDiagonalDistance = dist1.getDistance(); 
    float leftDiagonalDistance = 40;
    // Process the left diagonal distance, e.g., marking cells based on the distance
    if(leftDiagonalDistance <= max_distance){

      if(leftDiagonalDistance <= 6){
        int squares = leftDiagonalDistance/4;
        for(int i = 1; i <= squares; i++){
            occupancyGrid.markCell(RobotCurrentPositionX - i, RobotCurrentPositionY - i, CLEAR);
        }
        occupancyGrid.markCell(RobotCurrentPositionX - squares-1, RobotCurrentPositionY + squares-1, OCCUPIED);
      }
      else if (leftDiagonalDistance <= 10){
        int squares = leftDiagonalDistance/4;
        for(int i = 1; i <= squares; i++){
            occupancyGrid.markCell(RobotCurrentPositionX - i, RobotCurrentPositionY - i, CLEAR);
        }
        occupancyGrid.markCell(RobotCurrentPositionX - squares-1, RobotCurrentPositionY + squares-1, OCCUPIED);
      }
      else if (leftDiagonalDistance <= 14){
        int squares = leftDiagonalDistance/4;
        for(int i = 1; i <= squares; i++){
            occupancyGrid.markCell(RobotCurrentPositionX - i, RobotCurrentPositionY - i, CLEAR);
        }
        occupancyGrid.markCell(RobotCurrentPositionX - squares-1, RobotCurrentPositionY + squares-1, OCCUPIED);
      }
        return true;
    }
    // // Return to the original orientation
    turnSlightlyRight();

    // // Turn slightly right to check the right diagonal
    turnSlightlyRight();
    // // Assume getDistance reads the distance after the slight right turn
    // float rightDiagonalDistance = dist1.getDistance();
    float rightDiagonalDistance = 11;
    // // Process the right diagonal distance

    if(rightDiagonalDistance <= max_distance){

      if(rightDiagonalDistance <= 6){
        int squares = rightDiagonalDistance/4;
        for(int i = 1; i <= squares; i++){
            occupancyGrid.markCell(RobotCurrentPositionX - i, RobotCurrentPositionY + i, CLEAR);
        }
        occupancyGrid.markCell(RobotCurrentPositionX - squares-1, RobotCurrentPositionY + squares+1, OCCUPIED);
      }
      else if (rightDiagonalDistance <= 10){
        int squares = rightDiagonalDistance/4;
        for(int i = 1; i <= squares; i++){
            occupancyGrid.markCell(RobotCurrentPositionX - i, RobotCurrentPositionY + i, CLEAR);
        }
        occupancyGrid.markCell(RobotCurrentPositionX - squares-1, RobotCurrentPositionY + squares+1, OCCUPIED);
      }
      else if (rightDiagonalDistance <= 14){
        int squares = rightDiagonalDistance/4;
        for(int i = 1; i <= squares; i++){
            occupancyGrid.markCell(RobotCurrentPositionX - i, RobotCurrentPositionY + i, CLEAR);
        }
        occupancyGrid.markCell(RobotCurrentPositionX - squares-1, RobotCurrentPositionY + squares+1, OCCUPIED);

      }
        return true;
    }

    // // Return to the original orientation
    turnSlightlyLeft();

    // // Based on the distances, mark cells or decide the next action
    // // This is where you'd implement logic similar to what was discussed before

    return true; // Return true if clear path is found, false otherwise
}

void Robot::updateRobotXCoord(int x){
  RobotCurrentPositionX = x;
}

void Robot::updateRobotYCoord(int y){
  RobotCurrentPositionY = y;
}


