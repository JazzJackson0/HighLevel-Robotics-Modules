#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>

using namespace Eigen;

struct PointCloud {
	std::vector<VectorXf> points;
	std::vector<float> weights;
};

struct ControlCommand {
	float trans_vel; 
	float rot_vel;
};