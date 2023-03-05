#pragma once
#include <iostream>
#include <vector>
#include <cmath>

#include </usr/include/eigen3/Eigen/Dense>
#include </usr/include/eigen3/Eigen/src/Core/Matrix.h>

#include <cppad/cppad.hpp>

using namespace::CppAD;
using namespace::Eigen;
using std::vector;

struct odometry_reading {
	float RobotTranslation; 
	float RobotRotation;
};

enum f_type {UPDATE, OBSERVATION};

typedef struct state_vec StateVec;
typedef struct pose_plus PosePlus;
typedef struct odometry_reading OdometryReadng;
typedef enum f_type FunctionType;


class EKFSlam {

    private:
        VectorXf StateVector; // The vector containing the current robot pose and all landmark positions.
        Eigen::MatrixXf UpdateMappingFunction;
		Eigen::MatrixXf ObservationMappingFunction;
		Eigen::MatrixXf UpdateCovariance;
		Eigen::MatrixXf ObservationCovariance;
		Eigen::MatrixXf MotionCovariance;
		float time_interval;

		// Dimensions
		int PoseDimensions;
		int LandmarkDimensions;
		int NumOfLandmarks; // Number of Landmarks in the Map

		// 
		Eigen::MatrixXf HighDimension_H;
		Eigen::MatrixXf KalmanGain;
		Eigen::MatrixXf Identity;
		
		VectorXf InitialPosition;
		VectorXf PreviousPose;
		std::vector<AD<float>> Xg;
		std::vector<AD<float>> Yg;
		std::vector<AD<float>> Xh;
		std::vector<AD<float>> Yh;
		ADFun<float> UpdateFunction;		
		ADFun<float> ObservationFunction;	

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
		 * @brief Creates a Landmark Range & Bearing Estimation based on a given mathematical model / Observation Function.
		 * 				|||  This function currently assumes a 3D pose and 2D landmark position.
		 *
		 * @param robot_pose_est Estimated Robot Pose (the output of the Update Pose function)
		 * @param landmark_position_est Estimated Landmark Position (Obtained from......................)
		 *
		 * @return ** VectorXf Updated Landmark Data
		 */
		VectorXf GetEstimatedScan(VectorXf robot_pose_est, VectorXf landmark_position_est);



		/**
		 * * @brief Sets up the Update Function/Motion Model g() as a true set of equations 
		 * 			with unknown independent variables for input into the Jacobian Matrix G.
		 *
		 * @param MeasuredPose The Previous Pose that the odometry commands will be applied to.
		 * @param odom Odometry reading (translation velocity & rotation velocity) 
		 *
		 * @return ** void
		 */
		void BuildUpdateFunctionFor_G(VectorXf MeasuredPose, OdometryReadng odom);



		/**
		 * @brief Sets up the Observation Function h() as a true set of equations 
		 * 			with unknown independent variables for input into the Jacobian Matrix H.
		 *
		 * @return ** void
		 */
		void BuildObservationFunctionFor_H();



        /**
         * @brief Update the Mean with a new Predicted State and propagate the Covariance Matrix
         *          forward in time.
		 *
		 * @param current_pose The current Pose that will be updated while going through 
		 * 						the Prediction Step
		 * @param odom Odometry reading (translation velocity & rotation velocity) 
         * 
         * @return ** void 
         */
        void Prediction(VectorXf current_pose, OdometryReadng odom);



        /**
         * @brief Correct the Predicted state using the given Input data and 
         *          a Kalman Gain weight factor applied to the given estimate.
		 * 
		 * @param current_scan The current range scan.
		 * @param odom Odometry reading (translation velocity & rotation velocity) 
         * 
         * @return ** void 
         */
        void Correction(VectorXf current_scan, OdometryReadng odom);



        /**
         * @brief Creates and solves the Jacobian for a given function.
         * 
         * @param f_type The function to create and solve a Jacobian for.
		 * 					|||  UPDATE: Update Function, OBSERVATION: Observation Function
		 * 
		 * @param odom Odometry reading (translation velocity & rotation velocity) 
		 * 
         * @return ** MatrixXf - Jacobian Matrix 
         */
		MatrixXf CalculateJacobian(FunctionType f_type, OdometryReadng odom);
	


		/**
		 * @brief Builds the initial State Vector and Covariance Matrix
		 *
		 * @param landmarks Current vector of Landmarks
		 *
		 * @return ** void 
		 */
		void BuildUpdateCovarianceAndStateVector(std::vector<VectorXf> landmarks);

		/**
		 * @brief Get the Current Pose From State Vector
		 * 
		 * @return ** VectorXf - The Current Pose
		 */
		VectorXf GetPoseFromStateVector();
    
	public:

        /**
         * @brief Initialize an EKF Slam object
		 *
		 * @param initial_position Starting position of the robot
		 * @param map A vector of all landmark (x, y) positions. Known or Unknown.
		 * @param pose_dim Pose Dimensions
		 * @param landmark_dim Landmark Dimensions
		 * @param landmark_num Number of Landmarks
		 * @param obs_covariance Observation Covariance Matrix
		 * @param motion_covariance Expresses the uncertainty in the motion model.
         */
        EKFSlam(Eigen::VectorXf initial_position, std::vector<VectorXf> map, 
				int pose_dim, int landmark_dim, int landmark_num, 
				Eigen::MatrixXf obs_covariance, Eigen::MatrixXf motion_covariance);


       
		/**
         * @brief Run the EKF SLAM Algorithm
         * 
         * @return ** void 
         */
        void Run();
};


