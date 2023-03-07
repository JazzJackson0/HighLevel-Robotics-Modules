#include "AStar.hpp"


float A_Star::GetManhattanDistance(int currentRow, int currentCol, CellCoordinate goalCell) {
    int goalRow = goalCell.first;
    int goalCol = goalCell.second;
    // Returns H-Cost
    return (float)abs(currentRow - goalRow) + (float)abs(currentCol - goalCol);
}



float A_Star::GetDiagonalDistance(int currentRow, int currentCol, CellCoordinate goalCell) {

    int goalRow = goalCell.first;
    int goalCol = goalCell.second;
    float nodeDistance = 1;
    float nodeDiagonalDistance = sqrt(2);
    float dx = (float)abs(currentRow - goalRow);
    float dy = (float)abs(currentCol - goalCol);
    
    // Returns H-Cost
    return (float)(nodeDistance * (dx + dy) + (nodeDiagonalDistance - (2 * nodeDistance)) * 
        (float)fmin(dx, dy));
}



float A_Star::GetEuclideanDistance(int currentRow, int currentCol, CellCoordinate goalCell) {

    // Returns H-Cost
    return (float)(sqrt( pow( ( currentRow - goalCell.first ), 2 ) + 
        pow( ( currentCol - goalCell.second ), 2 ) ));
}



bool A_Star::isValid(int rowNum, int colNum) {
    return (rowNum >= 0) && (rowNum < G_ROW) && 
        (colNum >= 0) && (colNum < G_COL);
}



bool A_Star::isBlocked(int rowNum, int colNum) {
    return Grid[rowNum][colNum] == 1;
}



float A_Star::GetHVal(int rowNum, int colNum, CellCoordinate goalCell, DistanceFormula formula) {

    if (formula == MANHATTAN) { return GetManhattanDistance(rowNum, colNum, goalCell); }

    if (formula == DIAGONAL) { return GetDiagonalDistance(rowNum, colNum, goalCell); }

    if (formula == EUCLID) { return GetEuclideanDistance(rowNum, colNum, goalCell); }

    return 0.0;
}



bool A_Star::GoalReached(int x, int y, CellCoordinate goal) {

    return ( x == goal.first && y == goal.second);
}



Cell** A_Star::InitMatrixOfCells(int gridWith, int gridHeight, CellCoordinate startCell) {

    Cell **cells = (Cell**) malloc(gridWith * sizeof(Cell*));

    for (int i = 0; i < gridWith; i++) {
        cells[i] = (Cell*) malloc(gridHeight * sizeof(Cell));
    }

    for (int i = 0; i < gridWith; i++) {

        for (int j = 0; j < gridHeight; j++) {

            cells[i][j].FCost = FLT_MAX;
            cells[i][j].GCost = FLT_MAX;
            cells[i][j].HCost = FLT_MAX;
            cells[i][j].parentY = -1;
            cells[i][j].parentX = -1;
        }
    }

    //Inititalize the 'Start' Cell
    int x = startCell.first;
    int y = startCell.second;

    cells[x][y].FCost = 0.0;
    cells[x][y].GCost = 0.0;
    cells[x][y].HCost = 0.0;
    cells[x][y].parentY = x;
    cells[x][y].parentX = y;

    return cells;
}



bool** A_Star::InitBooleanMatrix(int gridWith, int gridHeight) {
    bool **bools = (bool**) malloc(gridWith * sizeof(bool*));

    for (int i = 0; i < gridWith; i++) {
        bools[i] = (bool*) malloc(gridHeight * sizeof(bool));
    }

    for (int i = 0; i < gridWith; i++) {

        for (int j = 0; j < gridHeight; j++) {

            bools[i][j] = false;
        }
    }
    return bools;
}



void A_Star::PathTrace(Cell **cells, CellCoordinate goalCell) {
    
    cout << "Path: " << endl;
    int x = goalCell.first;
    int y = goalCell.second;

    stack<CellCoordinate> Path;

    /* Follow the trail of Visited cells.
        When cell's current coordinates are the same as it's parent coordinaates, 
        you have reached the Starting cell.
    */
    while (!(cells[x][y].parentX == x && cells[x][y].parentY == y)) {
        
        // Push cell onto stack, then move to its parent
        Path.push(make_pair(x, y));
        int tempX = cells[x][y].parentX;
        int tempY = cells[x][y].parentY;
        x = tempX;
        y = tempY;
    }
    
    // Push Start cell onto stack
    Path.push(make_pair(x, y));

    // Pop and print each cell until stack is empty.
    while ( !(Path.empty()) ) {

        pair<int, int> p = Path.top();
        Path.pop();
        cout << "-> (" + p.first << ", " << p.second << ") " << endl;
    }

    return;
}



A_Star::A_Star(int **grid, int width, int height) {

    G_ROW = width;
    G_COL = height;
    Grid = grid;
}



bool A_Star::Search(CellCoordinate startCell, CellCoordinate goalCell) {

    /*-----------------------Make Initial Checks-----------------------*/
    if (!isValid(startCell.first, startCell.second)) {
        cout << "Invalid Starting Coordinates." << endl;
        return;
    }

    if (!isValid(goalCell.first, goalCell.second)) {
        cout << "Invalid Goal Coordinates." << endl;
        return;
    }

    if (isBlocked(startCell.first, startCell.second)) {
        cout << "Start Coodrdinates Blocked." << endl;
        return;
    }

    if (GoalReached(startCell.first, startCell.second, goalCell)) {
        cout << "Ummm... You're already there..." << endl;
    }

    /*-----------------------Initial Set-Up-----------------------*/
    // Initialize a Boolean Matrix to keep track of Visited Cells
    bool **VisitedCells = InitBooleanMatrix(G_ROW, G_COL);

    /*A Matrix of all the cells whose details have been uncovered during the A* Process
        Meaning: Every "Visited" cell along with their 8 peripheral "Viewed" cells.
    */
    Cell **CellsViewed = InitMatrixOfCells(G_ROW, G_COL, startCell);

    // ??
    set<pPair> openList;

    // Put Starting Node in "Open List" with F-Cost set to 0
    openList.insert(make_pair(0.0, make_pair(startCell.first, startCell.second)));

    bool goalReached = false;
    
    // Main Loop
    while (!openList.empty()) {

        float New_FCost, New_GCost, New_HCost;

        pPair p = *openList.begin();

        // Remove vertex from openList
        openList.erase(openList.begin());
        // Add the vertex to marekedVertices
        int x = p.second.first;
        int y = p.second.second;
        VisitedCells[x][y] = true;

        // Will hold the coordinate values of a "Visited" cell's 8 peripheral cells
        int newX, newY;

        for (int i = 0; i < 8; i++) {

            switch (i) {
                case 0: // North Cell: (x-1, y)
                    newX = x - 1;
                    newY = y;
                case 1: // South Cell (x+1, y)
                    newX = x + 1;
                    newY = y;
                case 2: // East Cell (x, y+1)
                    newX = x;
                    newY = y + 1;
                case 3: // West Cell (x, y-1)
                    newX = x;
                    newY = y - 1;
                case 4: // North-East Cell (x-1, y+1)
                    newX = x - 1;
                    newY = y + 1;
                case 5: // North-West Cell (x-1, y-1)
                    newX = x - 1;
                    newY = y - 1;
                case 6: // South-East Cell (x+1, y+1)
                    newX = x + 1;
                    newY = y + 1;
                case 7: // South-West Cell (x+1, y-1)
                    newX = x + 1;
                    newY = y - 1;
            }

            if (isValid(newX, newY)) {

                // If the Goal has been reached
                if (GoalReached(newX, newY, goalCell))  {
                    
                    CellsViewed[newX][newY].parentX = x;
                    CellsViewed[newX][newY].parentY = y;
                    cout << "You've reached the goal!" << endl;
                    PathTrace(CellsViewed, goalCell);
                    goalReached = true;
                    return;
                }

                // If Cell has not been Visited, and is not Blocked
                else if (!(VisitedCells[newX][newY]) && !(isBlocked(newX, newY)))  {
                    
                    New_GCost = CellsViewed[newX][newY].GCost + 1.0;
                    New_HCost = GetHVal(newX, newY, goalCell, EUCLID);
                    New_FCost = New_GCost + New_HCost;

                    // If this Cell's F-Cost has not been set yet, or it has but the New F-Cost Calculation is Smaller
                    if (CellsViewed[newX][newY].FCost == FLT_MAX || CellsViewed[newX][newY].FCost > New_FCost) {
                        
                        //???????
                        openList.insert(make_pair(New_FCost, make_pair(newX, newY)));

                        //Update Cell Details
                        CellsViewed[newX][newY].FCost = New_FCost;
                        CellsViewed[newX][newY].GCost = New_GCost;
                        CellsViewed[newX][newY].HCost = New_HCost;
                        CellsViewed[newX][newY].parentY = x;
                        CellsViewed[newX][newY].parentX = y;
                    }
                }

            }

        } // End For-Loop
    } // End Main-Loop


    if (!goalReached) { 
        cout << "Failed to Reach Goal." << endl; 
        return false;
    }

    return true;
}




/*
 * 			TO-DO
 * 			-----
 *  - Test Code
 *
 *  - 
 *  
 *  - 
 *  */







