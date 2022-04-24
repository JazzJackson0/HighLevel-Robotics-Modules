#pragma once
//#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <vector>
#include <cstddef>
#include <bits/c++config.h>

#include </usr/include/eigen3/Eigen/Sparse>
#include </usr/include/eigen3/Eigen/Dense> // Are both of these needed?
#include </usr/include/eigen3/Eigen/Cholesky>
#include </usr/include/eigen3/Eigen/src/Core/util/Constants.h>

#include <cppad/core/sparse_jac.hpp>
#include <cppad/cppad.hpp>
#include <cppad/utility/sparse_rc.hpp>
#include <cppad/utility/sparse_rcv.hpp>
#include <cppad/utility/sparse2eigen.hpp>

#include "../DataStructures/Graph.hpp"

using namespace CppAD;
using namespace::Eigen;
using std::vector;
using std::pair;

typedef struct pose_edge PoseEdge;
typedef struct odometry_reading OdometryReadng;
typedef std::vector<size_t> SizeVector;
typedef std::vector<double> ValueVector;
//typedef vector<float> FloatVector;
//typedef vector<VectorXf> VecctorXfVector;


class PoseGraphOptSLAM {

	int MaxStateVectorSize_N;
	int PoseDimensions;     
	int MaxEdges;
	float time_interval;
	VectorXf PreviousPose;
	std::vector<AD<float>> X;
	std::vector<AD<float>> Y;
	ADFun<float> ErrorFunction;
	int VariationAroundGuess;
	
	private:

		/**
		 * @brief  Returns a vector containing coordinate data, 
		 * 				orientation, translational and angular velocity.
		 *
		 * @retirn ** vector<float> Coordinate data, 
		 * 				Orientation, Translational & Angular velocity.
		 */
		std::vector<float> Get_PoseData(); 


		/**
		 * @brief Updates the pose based on a given mathematical motion model / Update Function.
		 *
		 * @param MeasuredPose The Previous Pose, that the odometry commands will be applied to.
		 * @param odom Odometry reading (translation velocity & rotation velocity) 
		 *
		 * @return ** VectorXf Updated Pose
		 */
		VectorXf UpdatePose(VectorXf MeasuredPose, OdometryReadng odom);

		/**
		 * @brief Output the difference between the measured pose and the predicted pose
		 * 			in vector form.
		 * 
		 * @param MeasuredPose The current pose
		 * @param odom Odometry command to be applied to the previous pose
		 * @return ** VectorXf The Error Vector
		 */
		VectorXf GetErrorVector(VectorXf MeasuredPose, OdometryReadng odom);



		/**
		 * @brief Computes the difference between the Measured Pose and
		 * 			the the Predicted Pose (The result of the Observation Function f(x)).
		 *
		 * 			||| Observation Function: The mathematical model of the robot's sensing capabilities.
		 * 			Computes the Predicted Pose of the robot. (i.e., what the
		 * 			robot expects to observe.)
		 *
		 * @param MeasuredPose 
		 * @param odom Odometry reading (translation velocity & rotation velocity) 
		 *
		 * @return ** void
		 */
		void BuildErrorFunction(VectorXf MeasuredPose, OdometryReadng odom);



        /**
         * @brief The Front End: Turns raw sensor data (and corrected poses from the Back End)
         *          into Edges/Constraints using the Iterative Closest Point Algorithm.
         * 
         * @return ** vector<VectorXf> - Holds all of the nodes (pose vectors)in the graph.
         */
		std::vector<VectorXf> ICP(void);



        /**
         * @brief The Back End: Uses the Edges/Constraints from the Front End to optimize
         *          the graph and return the corrected poses.
         * 
         * @param StateVector The vector of poses to be optimized.
		 * @param i_and_j Indices of the two poses to have their (Observation Based Edge) errors minimized
		 * @param odom Odometry reading (translation velocity & rotation velocity) 
		 * 
         * @return ** vector<VectorXf> Updated State Vector 
         */
		std::vector<VectorXf> Optimize(std::vector<VectorXf> StateVector, pair<int, int> i_and_j, OdometryReadng odom);


        /**
         * @brief Creates the H matrix and b vector for the Linear system
         *          needed to minimize the error.
         * 
         * @param StateVector The vector of poses to be optimized.
		 * @param odom Odometry reading (translation velocity & rotation velocity) 
		 * 
         * @return ** pair<Eigen::SparseMatrix<float, Eigen::RowMajor>, VectorXf> 
		 * 				A pair containing the H Matrix and b vector.
         */
		pair<Eigen::SparseMatrix<float, Eigen::RowMajor>, VectorXf> BuildLinearSystem(
				std::vector<VectorXf> StateVector, OdometryReadng odom);		
		

    public:

        /**
         * @brief Initialies a Pose Graph Optimization object.
		 *
		 * @param max_nodes Maximum number of nodes allowed in the Pose Graph
		 * @param pose_dimension The number of elements in the Pose Vector
         * @param independent_val_num
		 * @param guess_variation Variation around a given guess (used for the linearization of the error vector)
		 *
         */
        PoseGraphOptSLAM(int max_nodes, int pose_dimension, int independent_val_num, int guess_variation);


        /**
         * @brief Run the Pose Graph Optimization SLAM Algorithm.
         * 
         * @return ** void 
         */
        void Run() {
            
        }


};


struct pose_edge {
    VectorXf Pose; // Pose.
    MatrixXf NoiseInfoMatrix; // Encodes the uncertainty in the transformation to the Pose.
};

struct odometry_reading {
	float RobotTranslation; 
	float RobotRotation;
};


