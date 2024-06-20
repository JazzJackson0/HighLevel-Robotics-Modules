#include "FrontierExploration.hpp"

// NOT FINISHED!!!!!!!!!!!

// Private-----------------------------------------------------------------------------------------------------------------------------------

bool FrontierExplorer::isValid(int row, int col) {
    return (col >= 0) && (col < N) && 
        (row >= 0) && (row < M);
}

bool FrontierExplorer::isFrontier(VectorXf point) {

    int row = point[1]; // Change VectorXf to VectorXi!!!!!!!!!!!!
    int col = point[0]; // Change VectorXf to VectorXi!!!!!!!!!!!!
 
    // North Cell: (x-1, y)
    if (isValid(row, col - 1) && Map(row, col - 1) < 0.5) { return true; }

    // South Cell (x+1, y)
    if (isValid(row, col - 1) && Map(row, col + 1) < 0.5) { return true; }

    // East Cell (x, y+1)
    if (isValid(row, col - 1) && Map(row + 1, col) < 0.5) { return true; }

    // West Cell (x, y-1)
    if (isValid(row, col - 1) && Map(row - 1, col) < 0.5) { return true; }

    // North-East Cell (x-1, y+1)
    if (isValid(row, col - 1) && Map(row + 1, col - 1) < 0.5) { return true; }

    // North-West Cell (x-1, y-1)
    if (isValid(row, col - 1) && Map(row - 1, col - 1) < 0.5) { return true; }

    // South-East Cell (x+1, y+1)
    if (isValid(row, col - 1) && Map(row + 1, col + 1) < 0.5) { return true; }

    // South-West Cell (x+1, y-1)
    if (isValid(row, col - 1) && Map(row - 1, col + 1) < 0.5) { return true; }

    return false;

}

std::vector<VectorXf> FrontierExplorer::Get_AdjacentCells(VectorXf point) {

    // TODO: 
}


int FrontierExplorer::Get_CellStatus(VectorXf point, int map_frontier) {

    int row = point[1]; // Change VectorXf to VectorXi!!!!!!!!!!!!
    int col = point[0]; // Change VectorXf to VectorXi!!!!!!!!!!!!
    

    if (map_frontier == MAP_STATUS) 
        return CellStatusMap[row][col].map_status;


    else if (map_frontier == FRONTIER_STATUS)
        return CellStatusMap[row][col].frontier_status;
}



void FrontierExplorer::Update_CellStatus(VectorXf point, int map_frontier, int status) {

    int row = point[1]; // Change VectorXf to VectorXi!!!!!!!!!!!!
    int col = point[0]; // Change VectorXf to VectorXi!!!!!!!!!!!!
    

    if (map_frontier == MAP_STATUS) {
        
        if (status == xOPEN) 
            CellStatusMap[row][col].map_status = M_OPEN;

        else if (status == xCLOSED) 
            CellStatusMap[row][col].map_status = M_CLOSED;
        
    }

    else if (map_frontier == FRONTIER_STATUS) {

        if (status == xOPEN) 
            CellStatusMap[row][col].frontier_status = F_OPEN;

        else if (status == xCLOSED) 
            CellStatusMap[row][col].frontier_status = F_CLOSED;
    }
}


bool FrontierExplorer::Has_OpenSpaceNeighbor(VectorXf point) {

}


std::vector<std::vector<VectorXf>> FrontierExplorer::Detect_WavefrontFrontier(VectorXf robot_pose) {

    MapQueue.push(robot_pose);
    MapOpenList.push_back(robot_pose);
    std::vector<std::vector<VectorXf>> frontiers;
    Update_CellStatus(robot_pose, MAP_STATUS, xOPEN);


    while (!MapQueue.empty()) {

        VectorXf point = MapQueue.front();
        MapQueue.pop();

        if (Get_CellStatus(point, MAP_STATUS) == xCLOSED) { continue; }

        if (isFrontier(point)) {
            std::vector<VectorXf> new_frontier = Extract_Frontier2D(point);
            frontiers.push_back(new_frontier);
        }

        std::vector<VectorXf> neighbors = Get_AdjacentCells(point);

        for (int i = 0; i < neighbors.size(); i++) {

            if (Get_CellStatus(point, MAP_STATUS) == xNONE && Has_OpenSpaceNeighbor(neighbors[i])) {
                MapQueue.push(neighbors[i]);
                Update_CellStatus(neighbors[i], MAP_STATUS, xOPEN);
            }

        }
        Update_CellStatus(point, MAP_STATUS, xCLOSED);
    }

    return frontiers;
}

std::vector<VectorXf> FrontierExplorer::Extract_Frontier2D(VectorXf frontier_pt) {

    FrontierQueue.push(frontier_pt);
    FrontierOpenList.push_back(frontier_pt);
    std::vector<VectorXf> new_frontier;

    while (!FrontierQueue.empty()) {

        VectorXf point = FrontierQueue.front();
        FrontierQueue.pop();

        if (Get_CellStatus(point, MAP_STATUS) == xCLOSED || Get_CellStatus(point, FRONTIER_STATUS) == xCLOSED) { continue; }

        if (isFrontier(point)) {
            
            new_frontier.push_back(point);
            std::vector<VectorXf> neighbors = Get_AdjacentCells(point);

            for (int i = 0; i < neighbors.size(); i++) {

                if (Get_CellStatus(point, FRONTIER_STATUS) == xNONE && Get_CellStatus(point, MAP_STATUS) != xCLOSED) {
                    MapQueue.push(neighbors[i]);
                    Update_CellStatus(neighbors[i], FRONTIER_STATUS, xOPEN);
                }
            }
        }

        Update_CellStatus(point, FRONTIER_STATUS, xCLOSED);
    }

    return new_frontier;
}

// TODO: Will likely need to add code where you update list status to M_NONE or F_NONE ?????

// Public-----------------------------------------------------------------------------------------------------------------------------------
FrontierExplorer::FrontierExplorer() { }

FrontierExplorer::FrontierExplorer(Eigen::Tensor<float, 2> map) : Map(map) { 

    auto &d = Map.dimensions();
    M = d[0];
	N = d[1];

    // Build CallStatusMap
    CellStatusMap = new PointStatus*[M];
    for (int i = 0; i < M; i++) {
        CellStatusMap[i] = new PointStatus[N];
    }
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {

            MapStatus map_status;
            FrontierStatus frontier_status;
            map_status = M_NONE;
            frontier_status = F_NONE;
            PointStatus cell = PointStatus(map_status, frontier_status);
            CellStatusMap[i][j] = cell;
        }
    }
}


void FrontierExplorer::Explore(VectorXf robot_pose) {

    std::vector<std::vector<VectorXf>> frontiers = Detect_WavefrontFrontier(robot_pose);

    // Decide on frontier to visit

    // Prepare data for Path Planning algo

    // Update status of frontiers


    // Probably need to make list of frontiers global
}

