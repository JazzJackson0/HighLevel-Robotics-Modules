#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cppad/cppad.hpp>
using namespace::CppAD;
using namespace::Eigen;
using std::vector;

typedef struct state_vec StateVec;
typedef struct pose_plus PosePlus;
typedef struct odometry_reading OdometryReadng;
typedef enum f_type FunctionType;


class EKFSlam {

    private:
        VectorXf StateVector;
        MatrixXf UpdateMappingFunction;
		MatrixXf ObservationMappingFunction;
		MatrixXf UpdateCovariance;
		MatrixXf ObservationCovariance;
		float time_interval;

		// Dimensions
		int PoseDimensions;
		int LandmarkDimensions;
		int NumOfLandmarks;

		// 
		MatrixXf HighDimension_H;
		MatrixXf KalmanGain;
		MatrixXf Identity;
		
		StateVec InitialState;
		VectorXf PreviousPose;
		VecAD<float> Xg((size_t) PoseDimensions);
		VecAD<float> Yg(); // Assign correct size
		VecAD<float> Xh((size_t) PoseDimensions);
		VecAD<float> Yh(); // Assign correct size
		ADFun<float> UpdateFunction;		
		ADFun<float> ObservationFunction;	

		/**
		 * @brief Updates the pose based on a given mathematical motion model / Update Function.
		 *
		 * @param MeasuredPose 
		 * @param odom Odometry reading (translation velocity & rotation velocity) 
		 *
		 * @return ** VectorXf Updated Pose
		 */
		VectorXf UpdatePose(VectorXf MeasuredPose, OdometryReadng odom);



		/**
		 * @brief Updates Landmark Range & Bearing data based on a given mathematical model / Observation Function.
		 * 				|||  This function currently assumes a 3D pose and 2D landmark position.
		 *
		 * @param robot_pose_est Estimated Robot Pose
		 * @param landmark_position_est Estimated Landmark Position
		 *
		 * @return ** VectorXf Updated Landmark Data
		 */
		VectorXf UpdateLandmarkObservation(VectorXf robot_pose_est, VectorXf landmark_position_est);



		/**
		 * * @brief Sets up the Update Function/Motion Model g() as a true set of equations 
		 * 			with unknown independent variables for input into the Jacobian Matrix G.
		 *
		 * @param MeasuredPose 
		 * @param odom Odometry reading (translation velocity & rotation velocity) 
		 *
		 * @return ** void
		 */
		void BuildUpdateFunctionFor_G(PosePlus MeasuredPose, OdometryReadng odom);



		/**
		 * @brief Sets up the Observation Function h() as a true set of equations 
		 * 			with unknown independent variables for input into the Jacobian Matrix H.
		 *
		 * @param MeasuredPose 
		 *
		 * @return ** void
		 */
		void BuildObservationFunctionFor_H(PosePlus MeasuredPose);



        /**
         * @brief Update the Mean with a new Predicted State and propagate the Covariance Matrix
         *          forward in time.
		 *
		 * @param current_pose 
		 * @param odom Odometry reading (translation velocity & rotation velocity) 
         * 
         * @return ** void 
         */
        void Prediction(VectorXf current_pose, OdometryReadng odom);



        /**
         * @brief Correct the Predicted state using the given Input data and 
         *          a Kalman Gain weight factor applied to the given estimate.
         * 
         * @return ** void 
         */
        void Correction();



        /**
         * @brief Creates and solves the Jacobian for a given function.
         * 
         * @param f_type The function to create and solve a Jacobian for.
		 * 					|||  UPDATE: Update Function, OBSERVATION: Observation Function
		 * @param StateVector The vector of poses to be optimized.
         * @return ** MatrixXf - Jacobian Matrix 
         */
		MatrixXf CalculateJacobian(FunctionType f_type, StateVec StateVector);
	


		/**
		 * @brief Builds the initial State Vector and Covariance Matrix
		 *
		 * @param pose_dim Pose Dimensions
		 * @param landmark_dim Landmark Dimensions
		 * @param landmark_num Number of Landmarks
		 * @param pose Current Pose
		 * @param landmarks Current vector of Landmarks
		 *
		 * @return ** void 
		 */
		void BuildUpdateCovarianceAndStateVector(int pose_dim, int landmark_dim, int landmark_num, 
				VectorXf pose, std::vector<VectorXf> landmarks);
    
	public:

        /**
         * @brief Initialize an EKF Slam object
		 *
		 * @param initial_state 
		 * @param pose_dim Pose Dimensions
		 * @param landmark_dim Landmark Dimensions
		 * @param landmark_num Number of Landmarks
		 * @param obs_covariance Observation Covariance Matrix
         */
        EKFSlam(StateVec initial_state, int pose_dim, int landmark_dim, 
				int landmark_num, MatrixXf obs_covariance);


       
		/**
         * @brief Run the EKF SLAM Algorithm
         * 
         * @return ** void 
         */
        void Run();
};

struct odometry_reading {
	float RobotTranslation; 
	float RobotRotation;
};

struct pose_plus {	// REMOVE THIS
	VectorXf RobotPose;
	float RobotTranslation; 
	float RobotRotation;
};

// Current State of Robot & Map
struct state_vec {
	PosePlus CurrentPose;
	std::vector<VectorXf> Map;
	int LandmarkCount; // Number of Landmarks in the Map
	int PoseDimension; // Max Dimension of the Pose Vector
	int StateVecSize; // Number of elements in State Vector
};

enum f_type {UPDATE, OBSERVATION};
