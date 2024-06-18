#include "FrontierExploration.hpp"

// NOT FINISHED!!!!!!!!!!!

// Private-----------------------------------------------------------------------------------------------------------------------------------

void FrontierExplorer::Detect_WavefrontFrontier(VectorXf robot_pose) {

    MapQueue.push(robot_pose);
    MapOpenList.push_back(robot_pose);

    while (!MapQueue.empty()) {

        VectorXf point = MapQueue.front();
        MapQueue.pop();

        if (1) { continue; }

        if (2) {
            std::vector<VectorXf> new_frontier = Extract_Frontier2D(point);
        }

        for (int i = 0; i < 1; i++) {

            if (3) {

            }

        }
 
    }
}

std::vector<VectorXf> FrontierExplorer::Extract_Frontier2D(VectorXf frontier_pt) {

    FrontierQueue.push(frontier_pt);
    FrontierOpenList.push_back(frontier_pt);
    std::vector<VectorXf> new_frontier;

    while (!FrontierQueue.empty()) {

        VectorXf point = FrontierQueue.front();
        FrontierQueue.pop();

        if (1) { continue; }

        if (2) {
            
            new_frontier.push_back(point);

            for (int i = 0; i < 1; i++) {

                if (3) {

                }

            }
        }
 
    }
}


// Public-----------------------------------------------------------------------------------------------------------------------------------


FrontierExplorer::FrontierExplorer() {


}


void FrontierExplorer::Explore(VectorXf robot_pose) {

    Detect_WavefrontFrontier(robot_pose);
}

