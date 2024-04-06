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
    //loop from top again
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
        }
    }
    //if front isnt clear, do a diagional check


    
}
