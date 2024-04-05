#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "OccupancyGrid.h"
#include <Arduino.h>
#include <vector>

class PathPlanner {
public:
    PathPlanner(OccupancyGrid& grid);
    void planPath(int startX, int startY, int goalX, int goalY);
    // Add more methods as needed...
private:
    OccupancyGrid& grid;
};

#endif
