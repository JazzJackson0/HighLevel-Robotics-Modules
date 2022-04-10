#pragma once
//#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <vector>
#include <Eigen/Sparse>
#include <Eigen/Dense> // Are both of these needed?
#include <Eigen/Cholesky>
#include "../DataStructures/Graph.hpp"
#include <cppad/cppad.hpp>
#include <cppad/utility/sparse_rc.hpp>
#include <cppad/utility/sparse_rcv.hpp>
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
	int PoseDimension;     
	int MaxEdges;
	float time_interval;
	VectorXf PreviousPose;
	VecAD<float> X((size_t)PoseDimension);
	VecAD<float> Y(); // Assign correct size
	ADFun<float> ErrorFunction;
	
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
         * @return ** vector<VectorXf> Updated State Vector 
         */
		std::vector<VectorXf> Optimize(std::vector<VectorXf> StateVector);


        /**
         * @brief Creates the H matrix and b vector for the Linear system
         *          needed to minimize the error.
         * 
         * @param StateVector The vector of poses to be optimized.
         * @return ** pair<Eigen::SparseMatrix<float, Eigen::RowMajor>, VectorXf> 
         */
		pair<Eigen::SparseMatrix<float>, Eigen::RowMajor BuildLinearSystem(
				std::vector<VectorXf> StateVector);		
		

    public:

        /**
         * @brief Initialies a Pose Graph Optimization object.
		 *
		 * @param max_nodes Maximum number of nodes allowed in the Pose Graph
		 * @param pose_dimension The number of elements in the Pose Vector
         * @param independent_val_num
		 *
         */
        PoseGraphOptSLAM(int max_nodes, int pose_dimension, int independent_val_num);


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


