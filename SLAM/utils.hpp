#pragma once
#include <iostream>
#include <cmath>
#include </usr/include/eigen3/Eigen/Dense>
#include </usr/include/eigen3/Eigen/src/Core/Matrix.h>

using std::pair;
using namespace Eigen;

typedef pair<float, VectorXf> AngleAndAxis;


/**
 * @brief Convert rotation matrix to rotation angle and axis of rotation pair
 * 
 * @param R Rotation matrix
 * @return AngleAndAxis - rotation angle and axis
 */
AngleAndAxis RotationMatrix_to_Angle(MatrixXf R);


/**
 * @brief Convert rotation angle and axis to Rotation matrix
 * 
 * @param a_a rotation angle and axis
 * @return MatrixXf - Return rotation matrix
 */
MatrixXf Angle_to_RotationMatrix(AngleAndAxis a_a);