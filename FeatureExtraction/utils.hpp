#pragma once
#include <iostream>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>

using Eigen::VectorXf;
using Eigen::MatrixXf;
using namespace::Eigen;

struct PointCloud {
	std::vector<VectorXf> points;
	std::vector<float> weights;
};

struct RotationTranslation {
	MatrixXf rotation_matrix;
	VectorXf translation_vector;
	VectorXf center_mass;
	float weight;
};


/**
 * @brief Calculate the Greatest Common Denominator between two given denominators
 * 
 * @param a First denominator
 * @param b Second denominator
 * @return int Greatest Common Denominator
 */
int gcd (int a, int b);

/**
 * @brief Return the largest value between the two numbers
 * 
 * @param a First number
 * @param b Second number
 * @return int 
 */
int max(int a, int b);

/**
 * @brief Return the smallest value between the two numbers
 * 
 * @param a First number
 * @param b Second number
 * @return int 
 */
int min(int a, int b);