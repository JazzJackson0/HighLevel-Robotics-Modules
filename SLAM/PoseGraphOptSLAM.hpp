#pragma once
//#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <vector>
#include <cstddef>
#include <bits/c++config.h>
#include <cmath>

#include </usr/include/eigen3/Eigen/Sparse>
#include </usr/include/eigen3/Eigen/Dense> // Are both of these needed?
#include </usr/include/eigen3/Eigen/Cholesky>
#include </usr/include/eigen3/Eigen/src/Core/util/Constants.h>

#include <cppad/cppad.hpp>
#include <cppad/core/sparse_jac.hpp>
#include <cppad/utility/sparse_rc.hpp>
#include <cppad/utility/sparse_rcv.hpp>
#include <cppad/utility/sparse2eigen.hpp>

#include "../DataStructures/Graph.hpp"

using CppAD::AD;
using CppAD::NearEqual;
using CppAD::sparse_rc;
using CppAD::sparse_rcv;
using namespace::CppAD;
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

	int MaxPoses;
	int PoseDimensions;     
	float time_interval;
	VectorXf PreviousPose;
	std::vector<AD<float>> X; // x j-i, x ij, y j-i, y ij, theta i, theta j, theta ij
	std::vector<AD<float>> Y;
	ADFun<float> ErrorFunction;
	int VariationAroundGuess;
	float min_convergence_thresh;
	
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
		 * @brief Create a single State Vector from the Graph Nodes
		 * 
		 * @param poses The Graph Nodes
		 * @return ** VectorXf 
		 */
		VectorXf CreateStateVector(std::vector<VectorXf> poses);


		/**
		 * @brief Output the difference between measured pose transformation
		 * 			and predicted pose transformation.
		 * 
		 * @param Pose_i The ith Pose
		 * @param Pose_j The jth Pose
		 * @param MeasuredTranslatedVector Change in x, y and theta. Obtained from the given Edge
		 * @return ** VectorXf The Error Vector
		 */
		VectorXf GetErrorVector(VectorXf Pose_i, VectorXf Pose_j, VectorXf MeasuredTranslatedVector);



		/**
		 * @brief Computes the difference between the Measured Transformation and
		 * 			the the Predicted Transformation.
		 *
		 * 			||| Observation Function: The mathematical model of the robot's sensing capabilities.
		 * 			Computes the Predicted Pose of the robot. (i.e., what the
		 * 			robot expects to observe.)
		 *
		 * @return ** void
		 */
		void BuildErrorFunction();



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
         * @param Poses The vector of poses to be optimized. 
		 * 						(Corresponds to the Vertices of the Graph G{V})
		 * @param Edges The Graph Edges G{E}
		 * @param odom Odometry reading (translation velocity & rotation velocity) 
		 * 
         * @return ** VectorXf Updated State Vector 
         */
		VectorXf Optimize(std::vector<VectorXf> Poses, std::vector<PoseEdge> Edges, OdometryReadng odom);


        /**
         * @brief Creates the H matrix and b vector for the Linear system
         *          needed to minimize the error.
         * 
         * @param pose_i The ith Pose
		 * @param pose_j The jth Pose
		 * @param MeasuredTranslatedVector 
		 * @param edge_covariance Covariance for the given ij Edge
		 * @param odom Odometry reading (translation velocity & rotation velocity) 
		 * 
         * @return ** pair<Eigen::SparseMatrix<float, Eigen::RowMajor>, VectorXf> 
		 * 				A pair containing the H Matrix and b vector.
         */
		pair<Eigen::SparseMatrix<float, Eigen::RowMajor>, VectorXf> BuildLinearSystem(
				VectorXf pose_i, VectorXf pose_j, VectorXf MeasuredTranslatedVector, 
				MatrixXf edge_covariance, OdometryReadng odom);		
		

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
        void Run();


};


struct pose_edge {
    pair<int, int> PoseIndices; // Indices of the 2 Poses
	VectorXf TransformationMatrix; // Transformation Matrix.
    MatrixXf NoiseInfoMatrix; // Encodes the uncertainty in the transformation to the Pose.
};

struct odometry_reading {
	float RobotTranslation; 
	float RobotRotation;
};


