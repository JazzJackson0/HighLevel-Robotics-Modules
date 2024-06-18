#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <regex>
#include </usr/include/eigen3/Eigen/Dense>

using namespace Eigen;

/**
 * @brief 
 * 
 * @param s 
 * @param regx 
 * @return std::vector<std::string> 
 */
std::vector<std::string> split(const std::string& s, std::string regx);


struct PointCloud {
	std::vector<VectorXf> points;
	std::vector<float> weights;
};