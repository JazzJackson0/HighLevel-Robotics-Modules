#pragma once
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <stack>
#include <set>
#include <cfloat>
using std::pair;
using std::stack;
using std::make_pair;
using std::set;
using std::cout;
using std::endl;

typedef struct cell {
    int parentX, parentY;
    double FCost, GCost, HCost;
}Cell; 

class A_Star {

    private: 
        
        int G_ROW; // Grid Width
        int G_COL; // Grid Height
        int **Grid; // Grid to Search
        typedef enum { MANHATTAN, DIAGONAL, EUCLID }DistanceFormula;
        typedef pair<int, int> CellCoordinate;
        // (Below) pPair represents a Vertex (F-Cost, <x-coordinate, y-coordinate>)
        typedef pair<float, pair<int, int>> pPair;

        /**
         * @brief Get the Manhattan Distance to goal. (An Approximate Heuristic)
         *          |||  Abs(currentX - goalX) + Abs(currentY - goalY)
         *          ||| Use when your movement is restricted to 4 directions
         *              (right, left, up, down)
         * 
         * @param currentRow Current x coordinate
         * @param currentCol Current y coordinate
         * @param goalCell Coordinates of the Goal cell
         * 
         * @return ** float 
         */
        float GetManhattanDistance(int currentRow, int currentCol, CellCoordinate goalCell);

        /**
         * @brief Get the Diagonal Distance to goal. (An Approximate Heuristic)
         *          ||| Use when you can move in all 8 directions
         * 
         * @param currentRow Current x coordinate
         * @param currentCol Current y coordinate
         * @param goalCell Coordinates of the Goal cell
         * 
         * @return ** float 
         */
        float GetDiagonalDistance(int currentRow, int currentCol, CellCoordinate goalCell);

        /**
         * @brief Get the Euclidean Distance to goal. (An Approximate Heuristic)
         *          |||  Simply the distance between the current cell and the goal cell 
         *              using the Distance Formula.
         *          ||| For movement in any direction.
         * 
         * @param currentRow Current x coordinate
         * @param currentCol Current y coordinate
         * @param goalCell Coordinates of the Goal cell
         * 
         * @return ** float 
         */
        float GetEuclideanDistance(int currentRow, int currentCol, CellCoordinate goalCell);


        /**
         * @brief Checks whether or not the (x, y) coordinates of a given
         *          cell are valid.
         * 
         * @param rowNum Row Number of the cell
         * @param colNum Column Number of the cell
         * @return true - If Cell is Valid
         * @return false - If Cell is Not Valid
         */
        bool isValid(int rowNum, int colNum);

        /**
         * @brief Checks the Grid to see whether or not a given cell is blocked.
         * 
         * @param rowNum Row Number of the cell
         * @param colNum Column Number of the cell
         * @return true - If Cell is Blocked
         * @return false - If Cell is Not Blocked
         */
        bool isBlocked(int rowNum, int colNum);

        /**
         * @brief Calculate a given cell's H-Cost (i.e. the cell's distance from the goal cell)
         * 
         * @param rowNum Row Number of the current cell 
         * @param colNum Column Number of the current cell
         * @param goalCell The Goal Cell
         * @param formula Type of distance calculation formula
         * @return ** float 
         */
        float GetHVal(int rowNum, int colNum, CellCoordinate goalCell, DistanceFormula formula);

        /**
         * @brief Checks if given coordinates (x, y) match those of the Goal coordinates.
         * 
         * @param x - x coordinate
         * @param y - y coordinate
         * @param goal - Goal Coordinates
         * @return true - If match 
         * @return false - If No match
         */
        bool GoalReached(int x, int y, CellCoordinate goal);

        /**
         * @brief A 2D array of cells whose size matches the Grid which the 
         *          algorithm is being performed on.
         * 
         * @param gridWith - The number of row elements in the Matrix of Cells
         * @param gridHeight - Height of the column elements in the Matrix of Cells
         * @return ** Cell** 
         */
        Cell** InitMatrixOfCells(int gridWith, int gridHeight, CellCoordinate startCell);

        /**
         * @brief Initialize a boolean matrix representation of the Grid 
         *          with every element set to false.
         *          This will keep track of all visited cells, with those visited
         *          being set to true.
         * 
         * @param gridWith Width of the Grid to be searched.
         * @param gridHeight Height of the grid to be searched.
         * @return ** bool** 
         */
        bool** InitBooleanMatrix(int gridWith, int gridHeight);

        /**
         * @brief Traces the discovered path from Goal cell back to Start Cell,
         *          then prints the path from Start to Goal.
         * 
         * @param cells Holds 2D array of the Uncovered and Visited Cells.
         * @param goalCell Coordinates of the goal cell.
         * @return ** void 
         */
        void PathTrace(Cell **cells, CellCoordinate goalCell);

 
    public:

        /**
         * @brief Performs an A* search on a given Grid
         * 
         * @param grid The Grid that will be searched
         * @param width Width of the Grid
         * @param height Heigh of the Grid
         * 
         */
        A_Star(int **grid, int width, int height);


        /**
         * @brief Runs the A* Search algorithm
         * 
         * @param startCell Starting coordinates
         * @param goalCell Goal coordinates
         * @return ** void 
         */
        void Search(CellCoordinate startCell, CellCoordinate goalCell);

};






