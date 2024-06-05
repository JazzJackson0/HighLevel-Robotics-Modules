#pragma once
#include <iostream>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>

using Eigen::VectorXf;
using Eigen::MatrixXf;
using namespace::Eigen;

struct PointCloud {
	std::vector<VectorXf> Points;
	std::vector<float> Weights;
};

struct RotationTranslation {
	MatrixXf rotation_matrix;
	VectorXf translation_vector;
	VectorXf center_mass;
	float weight;
};





