#ifndef ROBOT_H
#define ROBOT_H

#include "MotorExtended.h"
#include "DistanceSensor.h"

class Robot {
public:
    Robot(int ain1, int ain2, int pwma, int stby, int bin1, int bin2, int pwmb, int dist_pin, TwoWire& i2c);

    void updateAll();
    void moveForward(int speed);
    void moveReverse();
    void reverseFromLeft();
    void reverseFromRight();
    void turnLeft();
    void turnRight();
    void scanLeftandRight();
    void brakeMotors();
    void getClearOfObject();
    void seedRound();
    void linearDrive(int ticks);
    void turn90Degrees(int direction);
    void forwardOneSquare();
    void leftOneSquare();
    void rightOneSquare();
    void testRound();
    void driveInSquare();


    MotorExtended motor1;
    MotorExtended motor2;
    DistanceSensor dist1;

private:
    int distPin;
    TwoWire& i2c;
    int targetTicks = 0; // Default initialization
    bool isTraveling = false; // Default initialization
    float square_len = 4.; //Each square in occupancy grid is 4x4 inches

    long distToEncoderTicks(float distance);
};

#endif
