#pragma once
//#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <vector>
#include <cstddef>
#include <chrono>
#include <bits/c++config.h>
#include <cmath>
#include <limits>
#include <cppad/cppad.hpp>
#include <cppad/utility/sparse2eigen.hpp>
#include "unsupported/Eigen/CXX11/Tensor"
#include <omp.h>
#include "../DataStructures/Graph.hpp"
#include "../ScanMatching/ICP.hpp"
#include "../Mapping/MapBuilder.hpp"
#include "utils.hpp"

#define ICP_POSE_DIM 2
#define ICP_ERR_DIM 3

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
	
	private:

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
		MapBuilder map_builder;
		Eigen::Tensor<float, 2> map_structure;
		Eigen::Tensor<float, 2> map_structure_mask;
		const int VIEW_RANGE = 600; // cm

		int previous_graph_size;
		int map_height;
		int map_width;
	

		/**
		 * @brief Uses a BFS to propagate accross the map, marking the spaces that the robot can see as "free"
		 * 
		 */
		void propagateFreeSpace();

		/**
		 * @brief Checks if a given cell is visible from the robot
		 * 
		 * @param x 
		 * @param y 
		 * @param x_robot 
		 * @param y_robot 
		 * @return true 
		 * @return false 
		 */
		bool isVisible(int x, int y, int x_robot, int y_robot);


		/**
		 * @brief 
		 * 
		 * @return Eigen::Tensor<float, 2> 
		 */
		Eigen::Tensor<float, 2> UpdateMap();

		/**
		 * @brief Takes 2 point clouds and determines the amount of overlap between them.
		 * 			Done by calculating the center point (using mean) of each 360 point cloud and returning
		 * 			the euclidean distance between them.
		 * 
		 * 
		 * @param cloud_a point cloud a
		 * @param cloud_b point cloud b
		 * @return float - The overlap distance.
		 */
		float Calculate_Overlap(PointCloud cloud_a, PointCloud cloud_b);


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
		 * @brief Update all vertices in Graph with new Transformation Matrices. 
		 * 		i.e. Convert from StateVector back to transformation matrices.
		 * 
		 */
		void ConvertStateVector();


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
         * @return Eigen::Tensor<float, 2> 
         */
        Eigen::Tensor<float, 2> Run(PointCloud current_landmarks);

		/**
		 * @brief 
		 * 
		 * @param height 
		 * @param width 
		 */
		void Set_MapDimensions(int height, int width);

		/**
		 * @brief 
		 * 
		 * @return VectorXf 
		 */
		VectorXf BroadcastCurrentPose();


		
};



