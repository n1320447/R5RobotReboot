#include "PathPlanner.h"
#include "Robot.h"

PathPlanner::PathPlanner(OccupancyGrid& grid) : grid(grid) {}


/*
    INPUT:startX: x coord of robot, startY: y coord of robot, goalX: x coord of button, goalY: y coord of button
    DESCRIPTION: Loop through a series of scans to find clear path forward. move forward or diag(in rights/lefts)
*/
void PathPlanner::planPath(Robot& myRobot, int startX, int startY, int goalX, int goalY) {


    //********************************* //attempt#1 for algo // **************************
 


    // // Implement your path planning logic here (e.g., A* algorithm)
    // //scan front first
    // //if front is clear 6" or more
    //     //move forward max possible distance
    //     //update new spot of robot
    // //if front not clear 6" or more
    //     //do diagonal scan to find a left or right path
    //         // travel left or right diagional, depending which is clear.
    // //if goal coordinate is near by
    //     // try to press button
    
    // //in the end loop from top again
    // if(myRobot.scanFront() == true){ // true if there is a path directly forward
    //     //there is at least 1 square in front we can travel to, travel to it.
    //     /*
    //         //a example mini grid of how coordinates appear. 
    //         {
    //         [{0,0},{0,1},{0,2},{0,3}],
    //         [{1,0},{1,1},{1,2},{1,3}],
    //         [{2,0},{2,1},{2,2},{2,3}]
    //         }
        
    //     */
    //     int RobotXCoordinate = myRobot.RobotCurrentPositionX;
    //     while(myRobot.occupancyGrid.getCellState(RobotXCoordinate--,myRobot.RobotCurrentPositionY) == CLEAR){
    //         //move robot forward
    //         myRobot.forwardOneSquare();
    //         //update current Robot Location on grid after each move
    //         myRobot.updateRobotXCoord(RobotXCoordinate); // should only need to update the X cartesian coordinate for front moving moves
    //     }
    // } else if(myRobot.scanFront() == false) {// there is not a object within the safeDistance aka the path is really clear
    //     //move forward one square
    //      myRobot.forwardOneSquare();
    //      myRobot.updateRobotXCoord(myRobot.RobotCurrentPositionX);
    // }
    // //if front isnt clear, do a diagional check
    // if(myRobot.scanDiagonals() == true){
    //     //there exists a diagonal path to take
    //     //try to take right path if possible
    //     Coordinates goalCoordinates = findFarthestRightClearSquare(myRobot);

    // }


    //********************************* //end of attempt#1 for algo // **************************




    //********************************* //attempt#2 for algo // **************************
 
   
    //best case scenario - path forward is completely clear per our sensor reading.
    if(myRobot.scanFront() == false) {// there is not a object within the safeDistance aka the path is really clear
        //move forward one square
         myRobot.forwardOneSquare();
         myRobot.updateRobotXCoord(myRobot.RobotCurrentPositionX);
    }
    else if(myRobot.scanFront() == true) { //there is an object detected within our safe distance
        //maybe need a check to make sure the detected object is not too close.
        int RobotXCoordinate = myRobot.RobotCurrentPositionX;
        if(myRobot.occupancyGrid.getCellState(RobotXCoordinate--,myRobot.RobotCurrentPositionY) == CLEAR){
            //
            //move forward
            while(myRobot.occupancyGrid.getCellState(RobotXCoordinate--,myRobot.RobotCurrentPositionY) == CLEAR){
                myRobot.forwardOneSquare();
                myRobot.updateRobotXCoord(RobotXCoordinate);
            }
        } else if (myRobot.occupancyGrid.getCellState(RobotXCoordinate--,myRobot.RobotCurrentPositionY) == OCCUPIED || myRobot.occupancyGrid.getCellState(RobotXCoordinate--,myRobot.RobotCurrentPositionY) == UNKNOWN){
            //need to look for a diagonal route, front is occupied or for some reason unknown
            myRobot.scanDiagonals(); //this will mark the clear squares at the diagnols
            //find the farthest right clear square
            Coordinates topRightCoordinates = findFarthestRightClearSquare(myRobot); // these coords are farthest right that are clear
            //turnright in place
            myRobot.turn90Degrees(1); //turn right
            myRobot.updateRobotDirection('E'); //update direction
                //scan *new* front to make sure its clear
                myRobot.scanFront();
                //if *new* front is clear
                if(myRobot.occupancyGrid.getCellState(myRobot.RobotCurrentPositionX,myRobot.RobotCurrentPositionY++) == CLEAR){
                    // move forward
                    myRobot.forwardOneSquare();
                    //turnRobot back to North direction (left)
                    myRobot.turn90Degrees(-1);
                    myRobot.updateRobotDirection('N');
                        //check if button coordinates are near robot
                        //else
                            //start whole loop again
                //if *new*front not clear
                } else{
                    myRobot.turn90Degrees(-1); // turn back to original north direction
                    myRobot.updateRobotDirection('N');
                }
            //turn back to original direction
            //turnleft in place
            myRobot.turn90Degrees(-1); //turn left
            myRobot.updateRobotDirection('W');
                //scan *new* front to make sure its clear
                myRobot.scanFront();
                //if *new* front is clear
                if(myRobot.occupancyGrid.getCellState(myRobot.RobotCurrentPositionX,myRobot.RobotCurrentPositionY--) == CLEAR){
                    // move forward
                    myRobot.forwardOneSquare();
                    
                    myRobot.turn90Degrees(1);
                    myRobot.updateRobotDirection('N');
                        //check if button coordinates are near robot
                        //else
                            //start whole loop again
                } else {
                    myRobot.turn90Degrees(1);
                    myRobot.updateRobotDirection('N');
                }
        }
    }


    //check if robot if near button 
    checkIfNearGoal(myRobot, goalX, goalY);
    //********************************* //end of attempt#2 for algo // **************************

    
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

void PathPlanner::checkIfNearGoal(Robot& myRobot, int goalX, int goalY) {
    // Assuming each square is defined as a 4x4 inch area and the robot's current position
    // is accurately tracked and updated
    const int closenessThreshold = 1; // Defines being in the adjacent square as "close"

    int currentX = myRobot.RobotCurrentPositionX;
    int currentY = myRobot.RobotCurrentPositionY;

    // Check if the robot's current position is within one square of the goal position
    if (abs(currentX - goalX) <= closenessThreshold && abs(currentY - goalY) <= closenessThreshold) {
        // If within one square, consider it close to the goal
        Serial.println("Robot is close to the goal!");
        // Here you could trigger any specific behavior for being close to the goal,
        // such as attempting to press the button or making a final adjustment to position
    } else {
        Serial.println("Robot is not close to the goal.");
        // If not close, you might continue with path planning or other actions
    }
}
