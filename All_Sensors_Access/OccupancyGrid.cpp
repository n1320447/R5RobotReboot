#include "OccupancyGrid.h"

OccupancyGrid::OccupancyGrid() {
    initializeGrid();
}

void OccupancyGrid::initializeGrid() {
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            grid[i][j] = UNKNOWN;
        }
    }
}

void OccupancyGrid::markCell(int x, int y, CellState state) {
    if (x >= 0 && x < width && y >= 0 && y < height) {
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
