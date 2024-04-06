#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "OccupancyGrid.h"
#include <Arduino.h>
#include <vector>

class Robot;

// Define the Coordinates struct in the header file for accessibility
struct Coordinates {
    int x;
    int y;
};

class PathPlanner {
public:
    PathPlanner(OccupancyGrid& grid);
    void planPath(Robot& myRobot, int startX, int startY, int goalX, int goalY);
    Coordinates findFarthestRightClearSquare(Robot& myRobot);
    // Add more methods as needed...
private:
    OccupancyGrid& grid;
    void checkIfNearGoal(Robot& myRobot, int goalX, int goalY);
};

#endif
