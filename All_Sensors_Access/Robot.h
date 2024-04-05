#ifndef ROBOT_H
#define ROBOT_H

#include <Wire.h>
#include "MotorExtended.h"
#include "DistanceSensor.h"
#include "OccupancyGrid.h"  // Include the OccupancyGrid class header
#include "PathPlanner.h"    // Include the PathPlanner class header

class Robot {
public:
    Robot(int ain1, int ain2, int pwma, int stby, int bin1, int bin2, int pwmb, int dist_pin, TwoWire& i2c);

    // Existing methods...
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

    // New methods for occupancy grid and path planning
    void initialize();            // Initializes the robot, including sensors and initial sweep
    void performInitialSweep();   // Performs the initial 360-degree sweep to populate the occupancy grid
    void planPathToGoal();        // Plans a path from the robot's current position to the goal
    void navigateToGoal();        // Navigates the robot along the planned path to the goal
    void updateOccupancyGrid();   // Updates the occupancy grid based on sensor readings
    void replanPath();            // Replans the path if obstacles are encountered or if the grid is updated
    void turnInCircle();


    MotorExtended motor1;
    MotorExtended motor2;
    DistanceSensor dist1;
    OccupancyGrid occupancyGrid;  // Occupancy grid instance
    void linearDrive(int ticks);
    void testRound();
    
    MotorExtended motor1;
    MotorExtended motor2;
    DistanceSensor dist1;

private:
    int distPin;
    TwoWire& i2c;
    PathPlanner pathPlanner;      // Path planner instance
    int targetTicks = 0; // Default initialization
    bool isTraveling = false; // Default initialization
};

#endif
