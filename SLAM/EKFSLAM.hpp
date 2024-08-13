#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

#include </usr/include/eigen3/Eigen/Dense>
#include </usr/include/eigen3/Eigen/src/Core/Matrix.h>
#include "/usr/include/eigen3/unsupported/Eigen/CXX11/Tensor"
#include <cppad/cppad.hpp>

#include "../FeatureExtraction/FeatureExtraction.hpp"
#include "../Mapping/MapBuilder.hpp"
#include "utils.hpp"

using namespace CppAD;
using namespace Eigen;
using std::vector;


enum f_type {PREDICTION, OBSERVATION};

// Why did I make these. What are they even from???????????????????????????????????????????????
typedef struct state_vec StateVec;
typedef struct pose_plus PosePlus;
typedef enum f_type FunctionType;


class EKFSlam {

    private:
        FeatureExtractor feature_extractor;
		MapBuilder map_builder;
		Eigen::Tensor<float, 2> map_structure;
		const int VIEW_RANGE = 600; // cm
		
		VectorXf StateVector; // The vector containing the current robot pose and all landmark positions.
		std::vector<VectorXf> Landmarks;
        Eigen::MatrixXf PredictionMappingFunction_F;
		Eigen::MatrixXf ObservationMappingFunction_F;
		Eigen::MatrixXf Covariance;
		Eigen::MatrixXf ProcessNoiseCovariance_R;
		Eigen::MatrixXf MeasurementCovariance_Q;
		float process_uncertainty_r;
		float measurement_uncertainty_q;
		float time_interval;

		// Dimensions
		int PoseDimensions;
		int LandmarkDimensions;
		int NumOfLandmarks; // current Number of Landmarks in the Map
		int MaxLandmarks;
		std::vector<Landmark> Correspondence;
		float SimilarityMargin;

		// 
		Eigen::MatrixXf HighDimension_G;
		Eigen::MatrixXf HighDimension_H;
		Eigen::MatrixXf KalmanGain;
		Eigen::MatrixXf Identity;
		
		VectorXf InitialPosition;
		VectorXf PreviousPose;
		std::vector<AD<float>> Xg;
		std::vector<AD<float>> Yg;
		std::vector<AD<float>> Xh;
		std::vector<AD<float>> Yh;
		ADFun<float> PredictionFunction;		
		ADFun<float> ObservationFunction;	


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
		 * @brief Builds the INITIAL State Vector.
		 *
		 * @return ** void 
		 */
		void Build_StateVector();


		/**
		 * @brief Builds the INITIAL Covariance Matrix.
		 * 
		 */
		void Build_Covariance();


		/**
		 * @brief Builds the Process Noise and Measurement Noise covariance matrices
		 * 
		 */
		void Build_NoiseCovariances();


		/**
		 * @brief Builds the Observation and Update mapping function matrices
		 * 
		 */
		void Build_MappingFunctions();


		/**
		 * @brief Builds the Identity Matrix
		 * 
		 */
		void Build_Identity();

		/**
		 * @brief Add a new landmark coordinates to the map (Increasing thre size of the state vector and all related matrices),
		 * 			and add landmark to Correspondences.
		 * 
		 * @param landmark 
		 * @return int 
		 */
		int UpdateMapAndResize(Landmark landmark);


		/**
		 * @brief Predicts the pose based on a given mathematical motion model / Update Function g().
		 *
		 * @param ctrl Odometry reading (translation velocity & rotation velocity) 
		 *
		 * @return ** VectorXf Updated Pose
		 */
		VectorXf PredictPose_g(ControlCommand ctrl);


		/**
		 * @brief Creates a Landmark Range & Bearing Estimation based on a given mathematical model / Observation Function h().
		 * 				|||  This function currently assumes a 2D pose and 2D landmark position.
		 *
		 * @param robot_pose_est Estimated Robot Pose (the output of the Update Pose function)
		 * @param landmark_position_est Estimated Landmark Position (Obtained from......................)
		 *
		 * @return ** VectorXf Updated Landmark Data
		 */
		VectorXf GetEstimatedLandmark_h(VectorXf robot_pose_est, Point landmark_position_est);


		/**
		 * * @brief Sets up the Prediction Function/Motion Model g() as a true set of equations 
		 * 			with unknown independent variables for input into the Jacobian Matrix G.
		 *
		 * @return ** void
		 */
		void BuildPredictionFunctionFor_G();


		/**
		 * @brief Sets up the Observation Function h() as a true set of equations 
		 * 			with unknown independent variables for input into the Jacobian Matrix H.
		 *
		 * @return ** void
		 */
		void BuildObservationFunctionFor_H();


		/**
         * @brief Creates and solves the Jacobian for a given function.
         * 
         * @param f_type The function to create and solve a Jacobian for.
		 * 					|||  UPDATE: Update Function, OBSERVATION: Observation Function
		 * 
         * @return ** MatrixXf - Jacobian Matrix 
         */
		MatrixXf CalculateJacobian(FunctionType f_type, int landmark_location);


		/**
         * @brief Update the Mean with a new Predicted State and propagate the Covariance Matrix
         *          forward in time.
		 *
		 * @param current_pose The current Pose that will be updated while going through 
		 * 						the Prediction Step
		 * @param ctrl Odometry reading (translation velocity & rotation velocity) 
         * 
         * @return ** void 
         */
        void Prediction(VectorXf current_pose, ControlCommand ctrl);



        /**
         * @brief Correct the Predicted state using the given Input data and 
         *          a Kalman Gain weight factor applied to the given estimate.
		 * 
		 * @param current_scan The current range scan.
         * 
         * @return ** void 
         */
        void Correction(std::vector<Landmark> current_scan);
    
	public:

		/**
		 * @brief Default constructor
		 * 
		 */
		EKFSlam();

        /**
         * @brief Initialize an EKF Slam object
		 *
		 * 
		 * @param pose_dim Pose Dimensions
		 * @param landmark_dim Landmark Dimensions
         */
        EKFSlam(int pose_dim, int landmark_dim);

		/**
		 * @brief Set the Initial State 
		 * 
		 * @param initial_position Starting position of the robot
		 * @param _process_uncertainty_r A constant that corresponds to the non-zero diagonal components of R (dim = dim(pose))
		 * @param _measurement_uncertainty_q A constant that corresponds to the non-zero components of Q (The uncertainty of each landmark)
		 */
		void SetInitialState(Eigen::VectorXf initial_position, float _process_uncertainty_r, float _measurement_uncertainty_q);

       
		/**
		 * @brief Run the EKF SLAM Algorithm
		 * 
		 * @param current_scan 
		 * @param current_pose 
		 * @param ctrl 
		 * @return Eigen::Tensor<float, 2> 
		 */
        Eigen::Tensor<float, 2> Run(PointCloud current_scan, VectorXf current_pose, ControlCommand ctrl);


		/**
		 * @brief Set the landmark data if correspondences are known.
		 * 
		 * @param landmarks A vector of all known landmark (x, y) positions.
		 */
		void SetKnownLandmarks(std::vector<VectorXf> landmarks);

		/**
		 * @brief 
		 * 
		 * @param height 
		 * @param width 
		 */
		void Set_MapDimensions(int height, int width);


		/**
		 * @brief Create a Map object
		 * 
		 * @return Eigen::Tensor<float, 2> 
		 */
		Eigen::Tensor<float, 2> UpdateMap();
};


