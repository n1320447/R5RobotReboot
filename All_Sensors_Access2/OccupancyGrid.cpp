#include "OccupancyGrid.h"

OccupancyGrid::OccupancyGrid() {
    grid = new CellState*[height];
    for(int i = 0; i < height; ++i) {
        grid[i] = new CellState[width];
    }
    initializeGrid();
}

void OccupancyGrid::initializeGrid() {
    Serial.println("entered init grid");
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            grid[i][j] = UNKNOWN;
        }
    }
}

void OccupancyGrid::markCell(int x, int y, CellState state) {
    Serial.println("outside marking cell:");
    if (x >= 0 && x < width && y >= 0 && y < height) {
        Serial.println("inside marking cell:");
        grid[y][x] = state;
    }
}

CellState OccupancyGrid::getCellState(int x, int y) const {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        return grid[y][x];
    }
    return UNKNOWN; // Default return if out-of-bounds
}

void OccupancyGrid::markAreaAs(int centerX, int centerY, CellState state) {
    int halfSize = robotSize / 2;
    for (int i = -halfSize; i <= halfSize; ++i) {
        for (int j = -halfSize; j <= halfSize; ++j) {
            markCell(centerX + j, centerY + i, state);
        }
    }
}

void OccupancyGrid::printGrid(){
      Serial.println("inside print");
        for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            switch (grid[i][j]) {
                case UNKNOWN:
                    Serial.print("?"); // Represent UNKNOWN with ?
                    break;
                case CLEAR:
                    Serial.print("."); // Represent CLEAR with .
                    break;
                case OCCUPIED:
                    Serial.print("#"); // Represent OCCUPIED with #
                    break;
            }
        }
        Serial.print(i);
        Serial.println(); // Move to the next line after printing each row
    }
}



OccupancyGrid::~OccupancyGrid() {
    for(int i = 0; i < height; ++i) {
        delete[] grid[i]; // Free each sub-array
    }
    delete[] grid; // Free the array of pointers
}
