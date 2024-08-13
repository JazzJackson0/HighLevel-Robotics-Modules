#include "FrontierExploration.hpp"

// Private-----------------------------------------------------------------------------------------------------------------------------------
void FrontierExplorer::Build_CellStatusMap() {

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

void FrontierExplorer::Build_RecursionMap(RecursionPoint **&RecursionMap) {

    RecursionMap = new RecursionPoint*[M];
    for (int i = 0; i < M; i++) {
        RecursionMap[i] = new RecursionPoint[N];
    }
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {

            RecursionPoint cell = RecursionPoint(false);
            RecursionMap[i][j] = cell;
        }
    }
}

bool FrontierExplorer::isValid(int row, int col) {
    return (col >= 0) && (col < N) && 
        (row >= 0) && (row < M);
}

bool FrontierExplorer::isFrontierPoint(VectorXi point) {

    int row = point[1];
    int col = point[0]; 

    // If Point is not an open space
    if (Map(row, col) >= 0.5) { return false; }

    int directions[8][2] = { {1, 0}, {0, 1}, {-1, 0}, {0, -1}, 
                             {1, 1}, {1, -1}, {-1, 1}, {-1, -1} };

    for (auto& dir : directions) {
        int ncol = col + dir[0];
        int nrow = row + dir[1];

        if (isValid(nrow, ncol) && Map(nrow, ncol) == 0.5) { return true; }
    }

    return false;

}

VectorXi FrontierExplorer::Get_Centroid(std::vector<VectorXi> frontier) {

    VectorXi centroid(2);
    float x_sum = 0;
    float y_sum = 0;
    for (int j = 0; j < frontier.size(); j++) {

        x_sum += frontier[j][0];
        y_sum += frontier[j][1];
    }

    centroid << x_sum / (frontier.size()), y_sum / (frontier.size());
    return centroid;
}

void FrontierExplorer::Get_AdjacentCells(VectorXi point, std::vector<VectorXi> &adjacents, RecursionPoint **RecursionMap) {

    adjacents.push_back(point);

    int row = point[1];
    int col = point[0]; 

    RecursionMap[col][row].added = true;

    int directions[8][2] = { {1, 0}, {0, 1}, {-1, 0}, {0, -1}, 
                             {1, 1}, {1, -1}, {-1, 1}, {-1, -1} };


    for (auto& dir : directions) {
        int ncol = col + dir[0];
        int nrow = row + dir[1];

        VectorXi direction(2);
        direction << nrow, ncol;

        if (isValid(nrow, ncol) && isFrontierPoint(direction) && !RecursionMap[ncol][nrow].added) { 
            Get_AdjacentCells(direction, adjacents, RecursionMap); 
        }
    }
    return;
}


int FrontierExplorer::Get_CellStatus(VectorXi point, int map_frontier) {

    int row = point[1];
    int col = point[0];
    
    if (map_frontier == MAP_STATUS) 
        return CellStatusMap[row][col].map_status;


    else if (map_frontier == FRONTIER_STATUS)
        return CellStatusMap[row][col].frontier_status;
}



void FrontierExplorer::Update_CellStatus(VectorXi point, int map_frontier, int status) {

    int row = point[1];
    int col = point[0];

    if (!isValid(row, col)) {
        std::cout << "Cell Out of Bounds [Cannot Find Frontier]" << std::endl;
        std::cout << row << ", "<< col << std::endl;
        std::cout << "M: " << M << " x  N: " << N << std::endl;
        return;
    }
    
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


bool FrontierExplorer::Has_OpenSpaceNeighbor(VectorXi point) {

    int row = point[1];
    int col = point[0];

    int directions[8][2] = { {1, 0}, {0, 1}, {-1, 0}, {0, -1}, 
                             {1, 1}, {1, -1}, {-1, 1}, {-1, -1} };

    for (auto& dir : directions) {
        int ncol = col + dir[0];
        int nrow = row + dir[1];

        if (isValid(nrow, ncol) && Map(nrow, ncol) < 0.5) { return true; }
    }

    return false;
}


std::vector<std::vector<VectorXi>> FrontierExplorer::Detect_WavefrontFrontier(VectorXi robot_index) {

    MapQueue.push(robot_index);
    MapOpenList.push_back(robot_index);
    std::vector<std::vector<VectorXi>> frontiers;
    Update_CellStatus(robot_index, MAP_STATUS, xOPEN);


    while (!MapQueue.empty()) {

        VectorXi point = MapQueue.front();
        MapQueue.pop();

        if (Get_CellStatus(point, MAP_STATUS) == xCLOSED) { continue; }

        if (isFrontierPoint(point)) {
            std::vector<VectorXi> new_frontier = Extract_Frontier2D(point);
            frontiers.push_back(new_frontier);
        }

        RecursionPoint **recursion_map;
        Build_RecursionMap(recursion_map);
        std::vector<VectorXi> neighbors;
        Get_AdjacentCells(point, neighbors, recursion_map);

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

std::vector<VectorXi> FrontierExplorer::Extract_Frontier2D(VectorXi frontier_pt) {

    FrontierQueue.push(frontier_pt);
    FrontierOpenList.push_back(frontier_pt);
    std::vector<VectorXi> new_frontier;

    while (!FrontierQueue.empty()) {

        VectorXi point = FrontierQueue.front();
        FrontierQueue.pop();

        if (Get_CellStatus(point, MAP_STATUS) == xCLOSED || Get_CellStatus(point, FRONTIER_STATUS) == xCLOSED) { continue; }

        if (isFrontierPoint(point)) {
            
            new_frontier.push_back(point);
            RecursionPoint **recursion_map;
            Build_RecursionMap(recursion_map);
            std::vector<VectorXi> neighbors;
            Get_AdjacentCells(point, neighbors, recursion_map);

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
    Build_CellStatusMap();
}


void FrontierExplorer::Load_MAP(Eigen::Tensor<float, 2> map) {

    Map = map;
    auto &d = Map.dimensions();
    M = d[0];
	N = d[1];
    Build_CellStatusMap();
}


VectorXi FrontierExplorer::FindFrontier(VectorXi robot_pose) {

    std::vector<std::vector<VectorXi>> frontiers = Detect_WavefrontFrontier(robot_pose);
    
    // Decide on frontier to visit----------
    // TODO: Currently only takes into account closest frontier, not mix between closest and largest
    VectorXi closest_centroid = robot_pose;
    float closest_dist = std::numeric_limits<float>::max();
    for (int i = 0; i < frontiers.size(); i++) {

        VectorXi centroid = Get_Centroid(frontiers[i]);
        float dist = std::sqrt(std::pow((centroid[0] - robot_pose[0]), 2) + std::pow((centroid[0] - robot_pose[0]), 2));
        if (dist < closest_dist) {
            closest_dist = dist;
            closest_centroid = centroid;
        }
    }

    std::cout << "Returning Frontier Point!" << std::endl;
    std::cout << "Robot Pose: " << robot_pose << std::endl;
    std::cout << "Closest Frontier: " << closest_centroid << std::endl;
    return closest_centroid;
}



