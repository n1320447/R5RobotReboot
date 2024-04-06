#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <Arduino.h>

enum CellState { UNKNOWN, CLEAR, OCCUPIED };

class OccupancyGrid {
public:
    OccupancyGrid();
    ~OccupancyGrid();
    void markCell(int x, int y, CellState state);
    CellState getCellState(int x, int y) const;
    void initializeGrid();
    void markAreaAs(int centerX, int centerY, CellState state); // New method to mark an area
    void printGrid();
    static const int width = 108/4; // 9 feet in inches 
    static const int height = 36/4; // 3 feet in inches
    static const int robotSize = 6; // Robot size (6"x6"), can be adjusted

private:

    // CellState grid[height][width];
    CellState** grid;
};



#endif
