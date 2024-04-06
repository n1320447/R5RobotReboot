#include "PathPlanner.h"
#include "Robot.h"

PathPlanner::PathPlanner(OccupancyGrid& grid) : grid(grid) {}


/*
    INPUT:startX: x coord of robot, startY: y coord of robot, goalX: x coord of button, goalY: y coord of button
    DESCRIPTION: Loop through a series of scans to find clear path forward. move forward or diag(in rights/lefts)
*/
void PathPlanner::planPath(Robot& myRobot, int startX, int startY, int goalX, int goalY) {
    // Implement your path planning logic here (e.g., A* algorithm)
    //scan front first
    //if front is clear 6" or more
        //move forward max possible distance
        //update new spot of robot
    //if front not clear 6" or more
        //do diagonal scan to find a left or right path
            // travel left or right diagional, depending which is clear.
    //if goal coordinate is near by
        // try to press button
    
    //in the end loop from top again
    if(myRobot.scanFront() == true){ // true if there is a path directly forward
        //there is at least 1 square in front we can travel to, travel to it.
        /*
            //a example mini grid of how coordinates appear. 
            {
            [{0,0},{0,1},{0,2},{0,3}],
            [{1,0},{1,1},{1,2},{1,3}],
            [{2,0},{2,1},{2,2},{2,3}]
            }
        
        */
        int RobotXCoordinate = myRobot.RobotCurrentPositionX;
        while(myRobot.occupancyGrid.getCellState(RobotXCoordinate--,myRobot.RobotCurrentPositionY) == CLEAR){
            //move robot forward
            myRobot.forwardOneSquare();
            //update current Robot Location on grid after each move
            myRobot.updateRobotXCoord(RobotXCoordinate); // should only need to update the X cartesian coordinate for front moving moves
        }
    } else if(myRobot.scanFront() == false) {// there is not a object within the safeDistance aka the path is really clear
        //move forward one square
         myRobot.forwardOneSquare();
         myRobot.updateRobotXCoord(myRobot.RobotCurrentPositionX);
    }
    //if front isnt clear, do a diagional check
    if(myRobot.scanDiagonals() == true){
        //there exists a diagonal path to take
        //try to take right path if possible
        Coordinates goalCoordinates = findFarthestRightClearSquare(myRobot);

    }

    //***************
    //***************
    //attempt#2 for algo
    //best case scenario - path forward is completely clear per our sensor reading.
    if(myRobot.scanFront() == false) {// there is not a object within the safeDistance aka the path is really clear
        //move forward one square
         myRobot.forwardOneSquare();
         myRobot.updateRobotXCoord(myRobot.RobotCurrentPositionX);
    }
    if(myRobot.scanFront() == true) { //there is an object detected within our safe distance
        //maybe need a check to make sure the detected object is not too close.
        if(myRobot.occupancyGrid.getCellState(RobotXCoordinate--,myRobot.RobotCurrentPositionY) == CLEAR){
            //move one square
        }else if (myRobot.occupancyGrid.getCellState(RobotXCoordinate--,myRobot.RobotCurrentPositionY) == OCCUPIED || myRobot.occupancyGrid.getCellState(RobotXCoordinate--,myRobot.RobotCurrentPositionY) == UNKNOWN){
            //need to look for a diagonal route, front is occupied or for some reason unknown
            myRobot.scanDiagonals(); //this will mark the clear squares at the diagnols
            //find the farthest right clear square
            Coordinates goalCoordinates = findFarthestRightClearSquare(myRobot); // these are farthest
            
        }
    }


    
}

Coordinates PathPlanner::findFarthestRightClearSquare(Robot& myRobot) {
    Coordinates farthestRightClear = {-1, -1}; // Initialize to an invalid position

    for (int y = 0; y < myRobot.occupancyGrid.height; ++y) {
        for (int x = myRobot.occupancyGrid.width - 1; x >= 0; --x) { // Iterate from right to left
            if (myRobot.occupancyGrid.getCellState(x, y) == CLEAR) {
                farthestRightClear.x = x;
                farthestRightClear.y = y;
                return farthestRightClear; // Return as soon as the farthest right clear square is found
            }
        }
    }

    return farthestRightClear; // Returns {-1, -1} if no clear square is found
}
