#ifndef ROBOT_H
#define ROBOT_H

#include "MotorExtended.h"
#include "DistanceSensor.h"
#include "OccupancyGrid.h"  // Include the OccupancyGrid class header
#include "PathPlanner.h"  

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
    void turn45Degrees(int direction);
    void forwardOneSquare();
    void leftOneSquare();
    void rightOneSquare();
    void testRound();
    void driveInSquare();
    bool scanFront();
    bool scanDiagonals();
    void turnSlightlyLeft(); // to be implemented if not already
    void turnSlightlyRight(); // to be implemented if not already
    void updateRobotXCoord(int x);
    void updateRobotYCoord(int y);
    void updateRobotDirection(char d);


    MotorExtended motor1;
    MotorExtended motor2;
    DistanceSensor dist1;
    OccupancyGrid occupancyGrid;
    int RobotCurrentPositionX = 25; // since our graph is a 2d array we start at the bottom of the 2darray..
    int RobotCurrentPositionY = 4;
    char RobotDirection = 'N'; //N = North, S = South, W = West, E = East

private:
    int distPin;
    TwoWire& i2c;
    int targetTicks = 0; // Default initialization
    bool isTraveling = false; // Default initialization
    int square_len = 4; //Each square in occupancy grid is 4x4 inches

    const float max_distance = 24; //inches
    const float bot_offset = 4; //Distance between center of bot and the distance sensor


    float straight_ranges[3] = {6.0, 10.0, 14.0};
    float diagonal_ranges[3] = {8.485, 14.14, 19.80};

    PathPlanner pathPlanner;  
    long distToEncoderTicks(float distance);

};

#endif
