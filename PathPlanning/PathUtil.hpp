#pragma once
#include <iostream>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>
using namespace Eigen;

class PathUtil {

    private:


    public:

        PathUtil();

        std::vector<VectorXf> SamplePath(std::vector<VectorXi> path);

        std::vector<VectorXf> SmoothPath(std::vector<VectorXi> path);
};



