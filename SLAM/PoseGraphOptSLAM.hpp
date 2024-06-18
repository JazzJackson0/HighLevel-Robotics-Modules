#pragma once
//#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <vector>
#include <cstddef>
#include <bits/c++config.h>
#include <cmath>

#include <cppad/cppad.hpp>
#include <cppad/utility/sparse2eigen.hpp>
#include "/usr/include/eigen3/unsupported/Eigen/CXX11/Tensor"
//#include <cppad/core/sparse_jac.hpp>
//#include <cppad/utility/sparse_rc.hpp>
//#include <cppad/utility/sparse_rcv.hpp>


//#include <eigen3/Eigen/Sparse>
//#include <eigen3/Eigen/Cholesky>
//#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/src/Core/util/Constants.h>

#include "../DataStructures/Graph.hpp"
#include "../ScanMatching/ICP.hpp"
#include "utils.hpp"

using namespace CppAD;
using namespace Eigen;
//using CppAD::AD;
//using CppAD::NearEqual;
//using CppAD::sparse_rc;
//using CppAD::sparse_rcv;


using std::vector;
using std::pair;

typedef std::vector<size_t> SizeVector;
typedef std::vector<float> ValueVector;

struct Pose {
	//int Index; // Pose's Graph Index
	MatrixXf TransformationMatrix; // Global Transformation Matrix. (w/ Current Position & Orientation)
	VectorXf pose;
	PointCloud Landmarks;
};

struct PoseEdge {
	MatrixXf TransformationMatrix; // Transformation Matrix.
    MatrixXf NoiseInfoMatrix; // Encodes the uncertainty in the transformation to the Pose.
};

struct HbResults {
	MatrixXf Hii;
	MatrixXf Hjj;
	MatrixXf Hij;
	MatrixXf Hji;
	VectorXf bi;
	VectorXf bj;
};


class PoseGraphOptSLAM {

	int MaxPoses_n;
	int CurrentPoses_n;
	int PoseDimensions;     
	bool InitialScan;
	VectorXf StateVector;
	std::vector<VectorXf> rotation_axes; // Used for converting from State Vector back to Transformation matrices
	Pose PreviousPose;
	std::vector<AD<float>> X; // x_j, x_i, y_j, y_i, theta_j, theta_i
	std::vector<AD<float>> Y;
	ADFun<float> ErrorFunction;
	int VariationAroundGuess;
	float min_convergence_thresh;
	Graph<Pose, PoseEdge> Pose_Graph;
	int max_iterations;

	// Used in Front End
	int NRecentPoses; 
	float ClosureDistance;
	PointCloud PreviousLandmarks;
	float OverlapTolerance;
	ICP icp;
	
	private:

		/**
		 * @brief Takes 2 point clouds and determines the amount of overlap between them.
		 * 			This is done by calculating the mean of each point cloud and returning
		 * 			the euclidean distance between those two mean points.
		 * 
		 * 			Not sure how robust this is but it seems like a good enough way to check for overlap
		 * 
		 * @param landmarks_a point cloud a
		 * @param landmarks_b point cloud b
		 * @return float - The overlap distance.
		 */
		float Calculate_Overlap(PointCloud landmarks_a, PointCloud landmarks_b);


		/**
		 * @brief Create a transformation matrix from given pose data (assumes a 3D pose [x, y, theta])
		 * 
		 * @param x x-position
		 * @param y y-position
		 * @param angle_axis angle and aaxis of rotation
		 * @return MatrixXf 
		 */
		MatrixXf VectorToTransformationMatrix(int x, int y, AngleAndAxis angle_axis);


		/**
		 * @brief Update the State Vector from the Graph Nodes' Transformation matrices received from Front End
		 * 
		 * @return ** pair<VectorXf, std::vector<VectorXf>> StateVector and coressponding rotation axes 
		 */
		void UpdateStateVector();


		/**
		 * @brief 
		 * 
		 * @param pose 
		 * @return true 
		 * @return false 
		 */
		bool CheckForLoopClosure(Pose pose);


		/**
		 * @brief 
		 * 
		 * @param pose 
		 * @param edge 
		 */
		void AddPoseToGraph(Pose pose, PoseEdge edge);


		/**
		 * @brief Output the difference between measured pose transformation
		 * 			and predicted pose transformation.
		 * 
		 * @param Pose_i The ith Pose
		 * @param Pose_j The jth Pose
		 * @param MeasuredTranslatedVector Translation & Rotation. Obtained from the given Edge between poses i and j
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
		void Build_ErrorFunction();


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
         * @return ** HbResults
         */
		HbResults Build_LinearSystem(
				VectorXf pose_i, VectorXf pose_j, VectorXf MeasuredTranslatedVector, 
				MatrixXf edge_covariance);



        /**
         * @brief The Front End: Turns raw sensor data (and corrected poses from the Back End)
         *          into Edges/Constraints using the Iterative Closest Point Algorithm.
		 * 
		 * @param current_landmarks Current Landmarks picked up by the most resent scan
         * 
         * @return ** bool - True if new loop closure made.
         */
		bool FrontEnd(PointCloud current_landmarks);



        /**
         * @brief The Back End: Uses the Edges/Constraints from the Front End to optimize
         *          the graph and return the corrected poses.
		 * 
         * @return ** void
         */
		void Optimize();


        		
    public:

		/**
         * @brief Default Constructor.
		 *
         */
        PoseGraphOptSLAM();

        /**
         * @brief Initialies a Pose Graph Optimization object.
		 *
		 * @param max_nodes Maximum number of nodes allowed in the Pose Graph
		 * @param pose_dimension The number of elements in the Pose Vector
		 * @param guess_variation Variation around a given guess (used for the linearization of the error vector)
		 *
         */
        PoseGraphOptSLAM(int max_nodes, int pose_dimension, int guess_variation);


		/**
		 * @brief 
		 * 
		 * @param n_recent_poses The number/amount of most recent poses to store.
		 * @param closure_distance The minimum distance between 2 poses required for loop closure
		 */
		void FrontEndInit(int n_recent_poses, float closure_distance);


        /**
         * @brief Run the Pose Graph Optimization SLAM Algorithm for 1 iteration.
         * 
         * @param current_landmarks 
		 * @param currentPose 
         */
        void Run(PointCloud current_landmarks, VectorXf &currentPose);


		/**
		 * @brief 
		 * 
		 * @return Eigen::Tensor<float, 2> 
		 */
		Eigen::Tensor<float, 2> CreateMap();
};



