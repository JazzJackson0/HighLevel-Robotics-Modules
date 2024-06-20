#pragma once
#include <iostream>
#include <queue>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>
#include "/usr/include/eigen3/unsupported/Eigen/CXX11/Tensor"
using namespace Eigen;

typedef enum {M_OPEN, M_CLOSED, M_NONE} MapStatus; 
typedef enum {F_OPEN, F_CLOSED, F_NONE} FrontierStatus; 
#define xOPEN 0
#define xCLOSED 1
#define xNONE 2

#define MAP_STATUS 33
#define FRONTIER_STATUS 44


struct PointStatus {
    MapStatus map_status;
    FrontierStatus frontier_status;
    PointStatus() {}
    PointStatus(MapStatus m_status, FrontierStatus f_status) : map_status(m_status), frontier_status(f_status) {}
};


class FrontierExplorer {

    private:
        Eigen::Tensor<float, 2> Map;
        int M;
        int N;
        PointStatus **CellStatusMap;
        std::queue<VectorXf> MapQueue;
        std::queue<VectorXf> FrontierQueue;
        std::vector<VectorXf> MapOpenList;
        std::vector<VectorXf> MapClosedList;
        std::vector<VectorXf> FrontierOpenList;
        std::vector<VectorXf> FrontierClosedList;

        bool isValid(int row, int col);

        bool isFrontier(VectorXf point);

        std::vector<VectorXf> Get_AdjacentCells(VectorXf point);

        int Get_CellStatus(VectorXf point, int map_frontier);

        void Update_CellStatus(VectorXf point, int map_frontier, int status);

        bool Has_OpenSpaceNeighbor(VectorXf point);

        std::vector<std::vector<VectorXf>> Detect_WavefrontFrontier(VectorXf robot_pose);

        std::vector<VectorXf> Extract_Frontier2D(VectorXf frontier_pt);

    public:

        FrontierExplorer();

        FrontierExplorer(Eigen::Tensor<float, 2> map);


        void Explore(VectorXf robot_pose);
};






