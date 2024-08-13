#include "Star.hpp"

float A_Star::Get_ManhattanDistance(int x1, int y1, int x2, int y2) {

    // Returns H-Cost
    return (float)abs(x1 - x2) + (float)abs(y1 - y2);
}



float A_Star::Get_DiagonalDistance(int x1, int y1, int x2, int y2) {

    float nodeDistance = 1;
    float nodeDiagonalDistance = sqrt(2);
    float dx = (float)abs(x1 - x2);
    float dy = (float)abs(y1 - y2);
    
    // Returns H-Cost
    return (float)(nodeDistance * (dx + dy) + (nodeDiagonalDistance - (2 * nodeDistance)) * 
        (float)fmin(dx, dy));
}


float A_Star::Get_EuclideanDistance(int x1, int y1, int x2, int y2) {

    // Returns H-Cost
    return (float)(sqrt( pow( ( x1 - x2 ), 2 ) + 
        pow( ( y1 - y2 ), 2 ) ));
}


bool A_Star::isValid(int x, int y) {
    return (x >= 0) && (x < WIDTH) && 
        (y >= 0) && (y < HEIGHT);
}



bool A_Star::isBlocked(int x, int y) {

    int row = y;
    int col = x;
    return MAP(row, col) > 0.5f;
}


bool A_Star::isGoalReached(int x, int y) {

    return ( x == Goal[0] && y == Goal[1]);
}


bool A_Star::isStartAndGoalValid() {
    
    if (!isValid(Start[0], Start[1])) {
        std::cout << "Invalid Starting Coordinates." << std::endl;
        return false;
    }

    if (!isValid(Goal[0], Goal[1])) {
        std::cout << "Invalid Goal Coordinates." << std::endl;
        return false;
    }

    if (isBlocked(Start[0], Start[1])) {
        std::cout << "Start Coodrdinates Blocked." << std::endl;
        return false;
    }

    if (isGoalReached(Start[0], Start[1])) {
        std::cout << "Ummm... You're already there..." << std::endl;
        return false;
    }

    return true;
}



float A_Star::Get_HCost(int x, int y, DistanceFormula formula) {

    int goal_x = Goal[0];
    int goal_y = Goal[1];

    if (formula == MANHATTAN) { return Get_ManhattanDistance(x, y, goal_x, goal_y); }

    if (formula == DIAGONAL) { return Get_DiagonalDistance(x, y, goal_x, goal_y); }

    if (formula == EUCLID) { return Get_EuclideanDistance(x, y, goal_x, goal_y); }

    return 0.0;
}


float A_Star::Get_GCost(int x1, int y1, int x2, int y2, DistanceFormula formula) {

    if (formula == MANHATTAN) { return Get_ManhattanDistance(x1, y1, x2, y2); }

    if (formula == DIAGONAL) { return Get_DiagonalDistance(x1, y1, x2, y2); }

    if (formula == EUCLID) { return Get_EuclideanDistance(x1, y1, x2, y2); }

    return 0.0;
}


astar::Cell A_Star::AReallyShitty_LowestCostNodeFunction(std::vector<astar::Cell> &uncovered) {

    float lowest_fcost = FLT_MAX;
    float lowest_hcost = FLT_MAX;
    int lowest_f_indx = -1;
    
    // Get Lowest FCost
    for (int i = 0; i < uncovered.size(); i++) {

        if (uncovered[i].FCost < lowest_fcost) {
            lowest_fcost = uncovered[i].FCost;
            lowest_f_indx = i;
        }
    }

    std::vector<int> same;
    for (int i = 0; i < uncovered.size(); i++) {
        if (uncovered[i].FCost == lowest_fcost) {
            same.push_back(i);
        }
    }

    // Get Lowest HCost
    int lowest_h_indx = lowest_f_indx;
    for (int i = 0; i < same.size(); i++) {

        if (uncovered[same[i]].HCost < lowest_hcost) {
            lowest_hcost = uncovered[same[i]].HCost;
            lowest_h_indx = same[i];
        }
    }

    astar::Cell cell = uncovered[lowest_h_indx];

    // Delete and adjust vector without using iterators
    for (int i = lowest_h_indx; i < uncovered.size() - 1; i++) {

        uncovered[i] = uncovered[i + 1];
    }
    uncovered.pop_back();

    //std::cout << "Cell Found: " << cell.x << ", " << cell.y << ")" << std::endl;
    return cell;
}


void A_Star::Get_AdjacentCellCoordinates(int x, int y, int &x_adjacent, int &y_adjacent, int adjacent_cell_num) {

    switch (adjacent_cell_num) {
        case 0: // North Cell: (x-1, y)
            x_adjacent = x - 1;
            y_adjacent = y;
            break;
        case 1: // South Cell (x+1, y)
            x_adjacent = x + 1;
            y_adjacent = y;
            break;
        case 2: // East Cell (x, y+1)
            x_adjacent = x;
            y_adjacent = y + 1;
            break;
        case 3: // West Cell (x, y-1)
            x_adjacent = x;
            y_adjacent = y - 1;
            break;
        case 4: // North-East Cell (x-1, y+1)
            x_adjacent = x - 1;
            y_adjacent = y + 1;
            break;
        case 5: // North-West Cell (x-1, y-1)
            x_adjacent = x - 1;
            y_adjacent = y - 1;
            break;
        case 6: // South-East Cell (x+1, y+1)
            x_adjacent = x + 1;
            y_adjacent = y + 1;
            break;
        case 7: // South-West Cell (x+1, y-1)
            x_adjacent = x + 1;
            y_adjacent = y - 1;
            break;
    }
}



astar::Cell** A_Star::Init_MatrixOfCells() {

    // 0. Allocate Memory for Map of Cells
    astar::Cell **cells = (astar::Cell**) malloc(WIDTH * sizeof(astar::Cell*));

    for (int i = 0; i < WIDTH; i++) {
        cells[i] = (astar::Cell*) malloc(HEIGHT * sizeof(astar::Cell));
    }

    // 1. Setup Map of Cells
    for (int i = 0; i < WIDTH; i++) {

        for (int j = 0; j < HEIGHT; j++) {

            cells[i][j].FCost = FLT_MAX - 1;
            cells[i][j].GCost = 0.f;
            cells[i][j].HCost = FLT_MAX - 1;
            cells[i][j].parentY = -1;
            cells[i][j].parentX = -1;
        }
    }

    // 2. Inititalize the 'Start' Cell
    int x = Start[0];
    int y = Start[1];

    cells[x][y].FCost = 0.0;
    cells[x][y].GCost = 0.0;
    cells[x][y].HCost = 0.0;
    cells[x][y].parentY = x;
    cells[x][y].parentX = y;

    return cells;
}



bool** A_Star::Init_BooleanMatrix() {
    
    // 0. Allocate Memory for Map of Booleans
    bool **bools = (bool**) malloc(WIDTH * sizeof(bool*));
    
    for (int i = 0; i < WIDTH; i++) {
        bools[i] = (bool*) malloc(HEIGHT * sizeof(bool));
    }

    // 1. Setup Map of Booleans
    for (int i = 0; i < WIDTH; i++) {

        for (int j = 0; j < HEIGHT; j++) {

            bools[i][j] = false;
        }
    }
    return bools;
}



void A_Star::PathTrace(astar::Cell **cells) {
    
    int x = Goal[0];
    int y = Goal[1];

    stack<VectorXi> Path;

    // Follow the trail of Visited cells. From GOAL to START
    // While Loop End Contition: When cell's current coordinates are the same as it's parent coordinaates, you have reached the Starting cell.
    while (!(cells[x][y].parentX == x && cells[x][y].parentY == y)) {
        
        //std::cout << "Cell: [" << x << ", " << y << "]" << std::endl;

        // Push cell onto stack, then move to its parent
        VectorXi cell(2);
        cell << x, y;
        Path.push(cell);
        int tempX = cells[x][y].parentX;
        int tempY = cells[x][y].parentY;
        x = tempX;
        y = tempY;
        
        //std::cout << "Parent: [" << x << ", " << y << "]" << std::endl;
        //std::cout << std::endl;
    }
    
    // Push Start cell onto stack
    VectorXi start_cell(2);
    start_cell << x, y;
    Path.push(start_cell);

    // Create the Path vector From START to GOAL
    while ( !(Path.empty()) ) {
        VectorXi cell = Path.top();
        ThePath.push_back(cell);
        Path.pop();
    }

    return;
}

A_Star::A_Star() {
    // Defualt Constructor
}

A_Star::A_Star(Eigen::Tensor<float, 2> map) : MAP(map) {

    auto &d = MAP.dimensions();
	WIDTH = d[1];
	HEIGHT = d[0];
    // std::cout << std::endl;
    // std::cout << "Map:" << std::endl;
    // std::cout << MAP << std::endl;
    // std::cout << std::endl;
}


void A_Star::Load_MAP(Eigen::Tensor<float, 2> map) {

    MAP = map;

    auto &d = MAP.dimensions();
	WIDTH = d[1];
	HEIGHT = d[0];
    // std::cout << std::endl;
    // std::cout << "Map:" << std::endl;
    // std::cout << MAP << std::endl;
    // std::cout << std::endl;
}


std::vector<VectorXi> A_Star::Path(VectorXi startCell, VectorXi goalCell) {

    ThePath.clear();
    Start = startCell;
    Goal = goalCell;
    if (!isStartAndGoalValid()) { 
        ThePath.push_back(startCell);
        std::cout << "Invalid Coordinates for START (" << startCell.transpose() << ") or GOAL (" << goalCell.transpose() << ")" << std::endl;
        return ThePath; 
    }

    bool **Visited = Init_BooleanMatrix();
    std::vector<astar::Cell> Uncovered;
    astar::Cell **MapOfCells = Init_MatrixOfCells();
    
    // MArk the starting node as "Uncovered" and "Visited"
    astar::Cell start;
    start.x = Start[0];
    start.y = Start[1];
    start.parentX = Start[0];
    start.parentY = Start[1];
    start.FCost = 0.f;
    MapOfCells[start.x][start.y] = start;
    Uncovered.push_back(start);
    
    while (!Uncovered.empty()) {

        float New_FCost, New_GCost, New_HCost;

        // Visit lowest cost node
        //std::cout << "To Visit Size: " << Uncovered.size() << std::endl;
        astar::Cell cell = AReallyShitty_LowestCostNodeFunction(Uncovered);
        //std::cout << "Lowest Cost: (" << cell.x << ", " << cell.y << ")" << std::endl; 
        
        Visited[cell.x][cell.y] = true;
        // std::cout << "(" << cell.x << ", " << cell.y << ")" << std::endl;

        // Look at the 8 Adjacent Cells
        int x_adjacent, y_adjacent;
        for (int i = 0; i < 8; i++) {

            Get_AdjacentCellCoordinates(cell.x, cell.y, x_adjacent, y_adjacent, i);

            if (!isValid(x_adjacent, y_adjacent)) { continue; }

            if (isBlocked(x_adjacent, y_adjacent)) { continue; }

            // Already Visited
            if ((Visited[x_adjacent][y_adjacent])) { continue; }

            // If the Goal has been reached
            if (isGoalReached(x_adjacent, y_adjacent)) {

                MapOfCells[x_adjacent][y_adjacent].parentX = cell.x;
                MapOfCells[x_adjacent][y_adjacent].parentY = cell.y;
                PathTrace(MapOfCells);
                
                // Tests
                // std::cout << std::endl;
                // std::cout << "Path:" << std::endl;
                // for (int i = 0; i < ThePath.size(); i++) {
                //     std::cout << ThePath[i].transpose() << std::endl;
                // }

                return ThePath;
            }

            // Re-Calculate Costs
            New_GCost = Get_GCost(x_adjacent, y_adjacent, cell.x, cell.y, EUCLID) + MapOfCells[cell.x][cell.y].GCost;
            New_HCost = Get_HCost(x_adjacent, y_adjacent, EUCLID);
            New_FCost = New_GCost + New_HCost;

            // If New F-Cost is Smaller than Old F-Cost 
            if (New_FCost < MapOfCells[x_adjacent][y_adjacent].FCost) {

                // Update Adjacent Cell and add it to 'Uncovered' list
                MapOfCells[x_adjacent][y_adjacent].FCost = New_FCost;
                MapOfCells[x_adjacent][y_adjacent].GCost = New_GCost;
                MapOfCells[x_adjacent][y_adjacent].HCost = New_HCost;
                MapOfCells[x_adjacent][y_adjacent].x = x_adjacent;
                MapOfCells[x_adjacent][y_adjacent].y = y_adjacent;
                MapOfCells[x_adjacent][y_adjacent].parentX = cell.x;
                MapOfCells[x_adjacent][y_adjacent].parentY = cell.y;

                astar::Cell cell_to_visit = MapOfCells[x_adjacent][y_adjacent];
                Uncovered.push_back(cell_to_visit);  

                // Test
                //std::cout << "Adding Adjacent For Future Visit: (" << x_adjacent << ", " << y_adjacent << ")" << std::endl;    
            }
        } 
    } 

    std::cout << "No viable path found." << std::endl;
    ThePath.push_back(startCell);
    return ThePath;
}



/*
 * 			TO-DO
 * 			-----
 *  - Incorporate Costmap to keep robot from hugging the edges
 *
 *  - 
 *  
 *  - 
 *  */







