#include "PathUtil.hpp"



PathUtil::PathUtil() {
    map_builder = new MapBuilder();
}

std::vector<VectorXi> PathUtil::SamplePath(std::vector<VectorXi> path) {

    // Very simple sampling process. Will probably make more robust later
    std::vector<VectorXi> waypoints;
    for (int i = 0; i < path.size(); i+=2)
        waypoints.push_back(path[i]);

    return waypoints;
}

std::vector<VectorXf> PathUtil::SmoothPath(std::vector<VectorXi> path) {
    
    std::vector<VectorXf> smoothed_waypoints;

    // 1. Convert from matrix indexes to cartesian coordinates
    for (auto& point : path)
        smoothed_waypoints.push_back(map_builder->DataStructureIndex_to_MapCoordinate(point.head<2>()));
    
    // 2. Smooth Points
    // .....

    return smoothed_waypoints;
}




