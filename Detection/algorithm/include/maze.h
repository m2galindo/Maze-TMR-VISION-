
#ifndef MAZE_H
#define MAZE_H

// Include all data structures
#include "coord.h"
#include "Tile.h"
#include "arrCustom.h"
#include "stack.h"
#include "heap.h"
#include "encoder.h"
#include "robot.h"


int calculateTargetOrientation(const coord& current, const coord& next);

// Rotates robot to target orientation using shortest rotation path
// Also calls detection() once before rotating
void rotateToOrientation(int targetOrientation, Tile* curr);

// Returns the opposite direction
TileDirection getOppositeDirection(TileDirection dir);

// Calculates the next coordinate based on current position and direction
coord getNextCoord(const coord& current, TileDirection direction);

// Checks if a coordinate has already been visited
// Returns true if found in visitedMap, false otherwise
bool isVisited(const coord& pos, arrCustom<coord>& visitedMap);

// Detects and handles victims at the current tile
// Checks for 3 victim types: harmed, stable, unharmed
void detection(Tile* curr);

class maze {
public:
    // Constructor
    maze();
    
    // Main entry point - initializes data structures and starts DFS exploration
    void run_algs();

private:

    // Depth-First Search: explores the entire maze systematically
    // Uses Dijkstra to navigate between tiles
    void dfs(arrCustom<coord>& visitedMap, arrCustom<Tile>& tiles, arrCustom<coord>& tilesMap);
    
    // Dijkstra's algorithm: finds shortest path between start and end
    // Considers walls, black tiles, and tile weights
    void dijkstra(coord& start, coord& end, arrCustom<coord>& tilesMap, arrCustom<Tile>& tiles);
    
    // Follows a pre-calculated path from Dijkstra
    // Handles rotation, movement, and victim detection
    void followPath(Stack& path, arrCustom<Tile>& tiles, arrCustom<coord>& tilesMap);
//helpers
    // Handles robot going up or down a ramp
    // Creates new tile on different level and connects levels
    void handleRamp(coord& current, arrCustom<coord>& visitedMap, 
                    arrCustom<Tile>& tiles, arrCustom<coord>& tilesMap);
    
    // Explores all 4 directions from current tile
    // Creates neighbor tiles, checks for walls, adds to unvisited stack
    // afterRamp flag: true when called after handling a ramp (skips backward direction)
    void exploreNeighbors(coord& current, Tile* currentTile, Stack& unvisited,
                         arrCustom<coord>& visitedMap, arrCustom<Tile>& tiles, 
                         arrCustom<coord>& tilesMap, bool afterRamp = false);
};

#endif // MAZE_H