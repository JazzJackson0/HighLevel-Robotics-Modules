#pragma once
#include <iostream>
#include <queue>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include "unsupported/Eigen/CXX11/Tensor"

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

// struct RecursionPoint {
//     bool added;
//     RecursionPoint() {}
//     RecursionPoint(bool _added) : added(_added) {}
// };


class FrontierExplorer {

    private:
        Eigen::Tensor<float, 2> Map;
        int M;
        int N;
        PointStatus **CellStatusMap;
        std::queue<VectorXi> MapQueue;
        std::queue<VectorXi> FrontierQueue;
        std::vector<VectorXi> MapOpenList;
        std::vector<VectorXi> MapClosedList;
        std::vector<VectorXi> FrontierOpenList;
        std::vector<VectorXi> FrontierClosedList;

        /**
         * @brief 
         * 
         */
        void Build_CellStatusMap();

        // /**
        //  * @brief 
        //  * 
        //  * @param RecursionMap 
        //  */
        // void Build_RecursionMap(RecursionPoint **&RecursionMap);

        /**
         * @brief 
         * 
         * @param row 
         * @param col 
         * @return true 
         * @return false 
         */
        bool isValid(int row, int col);

        /**
         * @brief 
         * 
         * @param point 
         * @return true 
         * @return false 
         */
        bool isFrontierPoint(VectorXi point);

        /**
         * @brief 
         * 
         * @param point 
         * @return true 
         * @return false 
         */
        bool Has_OpenSpaceNeighbor(VectorXi point);

        /**
         * @brief 
         * 
         * @param frontier 
         * @return VectorXi 
         */
        VectorXi Get_Centroid(std::vector<VectorXi> frontier);

        // /**
        //  * @brief 
        //  * 
        //  * @param point 
        //  * @param adjacents 
        //  * @param RecursionMap 
        //  */
        // void Get_AdjacentCells(VectorXi point, std::vector<VectorXi> &adjacents, RecursionPoint **RecursionMap);

        /**
         * @brief 
         * 
         * @param point 
         * @param map_frontier 
         * @return int 
         */
        int Get_CellStatus(VectorXi point, int map_frontier);

        /**
         * @brief 
         * 
         * @param point 
         * @param map_frontier 
         * @param status 
         */
        void Update_CellStatus(VectorXi point, int map_frontier, int status);

        
        /**
         * @brief 
         * 
         * @param robot_index 
         * @return std::vector<std::vector<VectorXi>> 
         */
        std::vector<std::vector<VectorXi>> Detect_WavefrontFrontier(VectorXi robot_index);

        
        /**
         * @brief 
         * 
         * @param frontier_pt 
         * @return std::vector<VectorXi> 
         */
        std::vector<VectorXi> Extract_Frontier2D(VectorXi frontier_pt);

    public:

        /**
         * @brief Construct a new Frontier Explorer object
         * 
         */
        FrontierExplorer();

        /**
         * @brief Construct a new Frontier Explorer object
         * 
         * @param map 
         */
        FrontierExplorer(Eigen::Tensor<float, 2> map);


        /**
         * @brief 
         * 
         * @param map 
         */
        void Load_MAP(Eigen::Tensor<float, 2> map);

        /**
         * @brief 
         * 
         * @param robot_pose 
         * @return VectorXi - Frontier Centroid
         */
        VectorXi FindFrontier(VectorXi robot_pose);
};






