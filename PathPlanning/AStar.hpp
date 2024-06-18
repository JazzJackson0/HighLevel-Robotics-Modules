#pragma once
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <stack>
#include <queue>
#include <cfloat>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>
#include "/usr/include/eigen3/unsupported/Eigen/CXX11/Tensor"
using std::pair;
using std::stack;
using std::make_pair;
using std::vector;
using std::queue;
using namespace Eigen;

namespace astar {

struct Cell {
    // int x, y, parentX, parentY;
    // double FCost, GCost, HCost;

    int x;
    int y;
    int parentX;
    int parentY;
    double FCost;
    double GCost;
    double HCost;
}; 
};


class A_Star {

    private: 
        
        int WIDTH; // Grid Width
        int HEIGHT; // Grid Height
        Eigen::Tensor<float, 2> MAP; // Grid to Search
        VectorXi Goal;
        VectorXi Start;
        std::vector<VectorXi> ThePath;
        typedef enum { MANHATTAN, DIAGONAL, EUCLID }DistanceFormula;


        /**
         * @brief Get the Manhattan Distance to goal. (An Approximate Heuristic)
         *          |||  Abs(currentX - goalX) + Abs(currentY - goalY)
         *          ||| Use when your movement is restricted to 4 directions
         *              (right, left, up, down)
         * 
         * @param x1 Current x1 coordinate
         * @param y1 Current y1 coordinate
         * @param x2 Current x2 coordinate
         * @param y2 Current y2 coordinate
         * 
         * @return ** float 
         */
        float Get_ManhattanDistance(int x1, int y1, int x2, int y2);

        /**
         * @brief Get the Diagonal Distance to goal. (An Approximate Heuristic)
         *          ||| Use when you can move in all 8 directions
         * 
         * @param x1 Current x1 coordinate
         * @param y1 Current y1 coordinate
         * @param x2 Current x2 coordinate
         * @param y2 Current y2 coordinate
         * 
         * @return ** float 
         */
        float Get_DiagonalDistance(int x1, int y1, int x2, int y2);

        /**
         * @brief Get the Euclidean Distance to goal. (An Approximate Heuristic)
         *          |||  Simply the distance between the current cell and the goal cell 
         *              using the Distance Formula.
         *          ||| For movement in any direction.
         * 
         * @param x1 Current x1 coordinate
         * @param y1 Current y1 coordinate
         * @param x2 Current x2 coordinate
         * @param y2 Current y2 coordinate
         * 
         * @return ** float 
         */
        float Get_EuclideanDistance(int x1, int y1, int x2, int y2);


        /**
         * @brief Checks whether or not the (x, y) coordinates of a given
         *          cell are valid.
         * 
         * @param x Row Number of the cell
         * @param y Column Number of the cell
         * @return true - If Cell is Valid
         * @return false - If Cell is Not Valid
         */
        bool isValid(int x, int y);

        /**
         * @brief Checks the Grid to see whether or not a given cell is blocked.
         * 
         * @param x Row Number of the cell
         * @param y Column Number of the cell
         * @return true - If Cell is Blocked
         * @return false - If Cell is Not Blocked
         */
        bool isBlocked(int x, int y);


        /**
         * @brief Checks if given coordinates (x, y) match those of the Goal coordinates.
         * 
         * @param x - x coordinate
         * @param y - y coordinate
         * @return true - If match 
         * @return false - If No match
         */
        bool isGoalReached(int x, int y);


        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool isStartAndGoalValid();


        /**
         * @brief Calculate a given cell's H-Cost (i.e. the cell's distance from the goal cell)
         * 
         * @param x Row Number of the current cell 
         * @param y Column Number of the current cell
         * @param formula Type of distance calculation formula
         * @return ** float 
         */
        float Get_HCost(int x, int y, DistanceFormula formula);


        /**
         * @brief Calculate a given cell's G-Cost (i.e. How far away that node is 
         *          from the starting node based on the CURRENT path between them)
         * 
         * @param x1 Current x1 coordinate
         * @param y1 Current y1 coordinate
         * @param x2 Current x2 coordinate
         * @param y2 Current y2 coordinate
         * @param formula Type of distance calculation formula
         * @return float 
         */
        float Get_GCost(int x1, int y1, int x2, int y2, DistanceFormula formula);

        
        /**
         * @brief Out of a list of all possible cells to visit next, 
         *      this function calculated the one with the lowest overall cost.
         *      It just so happens to do it in a terribly inefficient way
         * 
         * @param uncovered 
         * @return astar::Cell 
         */
        astar::Cell AReallyShitty_LowestCostNodeFunction(std::vector<astar::Cell> &uncovered);


        /**
         * @brief 
         * 
         * @param x 
         * @param y 
         * @param x_adjacent 
         * @param y_adjacent 
         * @param adjacent_cell_num 
         */
        void Get_AdjacentCellCoordinates(int x, int y, int &x_adjacent, int &y_adjacent, int adjacent_cell_num);


        /**
         * @brief A 2D array of Cell structs whose size matches the MAP the 
         *          algorithm is being performed on.
         * 
         * @return ** Cell** 
         */
        astar::Cell** Init_MatrixOfCells();

        /**
         * @brief Initialize a boolean matrix representation of the Grid 
         *          with every element set to false.
         *          This will keep track of all visited cells, with those visited
         *          being set to true.
         * 
         *          Holds all the cells whose details have been uncovered during the A* Process
         *          Meaning: Every "Visited" cell along with their 8 peripheral "Viewed" cells.
         * 
         * @return ** bool** 
         */
        bool** Init_BooleanMatrix();

        /**
         * @brief Traces the discovered path from Goal cell back to Start Cell,
         *          then prints the path from Start to Goal.
         * 
         * @param cells Holds 2D array of the Uncovered and Visited Cells.
         * @return ** void 
         */
        void PathTrace(astar::Cell **cells);

 
    public:

        /**
         * @brief Construct a new a star object
         * 
         */
        A_Star();

        /**
         * @brief Performs an A* search on a given Grid
         * 
         * @param map The Grid that will be searched
         * 
         */
        A_Star(Eigen::Tensor<float, 2> map);


        /**
         * @brief 
         * 
         * @param map 
         */
        void Load_MAP(Eigen::Tensor<float, 2> map);


        /**
         * @brief Runs the A* Search algorithm
         * 
         * @param startCell Starting coordinates
         * @param goalCell Goal coordinates
         * @return ** std::vector<VectorXf> The waypoints of the path 
         */
        std::vector<VectorXi> Path(VectorXi startCell, VectorXi goalCell);

};






