#pragma once
#include <iostream>
#include <queue>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>
using namespace Eigen;

typedef enum {OPEN, CLOSED} MapStatus; 
typedef enum {OPEN, CLOSED} FrontierStatus; 

struct Map_Pt {

    VectorXf point;
    MapStatus map_status;
    FrontierStatus frontier_status;
};

// Might need to take landmarks as input (From Frontier Extraction Algo) instead of raw point cloud ????

class FrontierExplorer {

    private:
        std::queue<VectorXf> MapQueue;
        std::queue<VectorXf> FrontierQueue;
        std::vector<VectorXf> MapOpenList;
        std::vector<VectorXf> MapClosedList;
        std::vector<VectorXf> FrontierOpenList;
        std::vector<VectorXf> FrontierClosedList;

        void Detect_WavefrontFrontier(VectorXf robot_pose);

        std::vector<VectorXf> Extract_Frontier2D(VectorXf frontier_pt);

    public:

        FrontierExplorer();


        void Explore(VectorXf robot_pose);
};






