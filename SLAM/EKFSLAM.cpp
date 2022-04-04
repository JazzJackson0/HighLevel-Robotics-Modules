#include "EKFSLAM.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>

VectorXf EKFSlam::UpdatePose(VectorXf MeasuredPose, float translation, float rotation) {
	
	VectorXf UpdatedPose(PoseDimensions);
	UpdatedPose[0] = (MeasuredPose[0] + (-1*(translation / rotation)) * CppAD::sin(Xg[2]) 
			+ (-1*(translation / rotation)) * CppAD::sin(Xg[2] + rotation * time_interval) );
	UpdatedPose[1] = (MeasuredPose[1] + (-1*(translation / rotation)) * CppAD::cos(Xg[2]) 
			+ (-1*(translation / rotation)) * CppAD::cos(Xg[2] + rotation * time_interval) );
	UpdatedPose[2] = (MeasuredPose[2] + rotation * time_interval);

	return UpdatedPose;
}



VectorXf EKFSlam::UpdateLandmarkObservation(VectorXf robot_pose_est, VectorXf landmark_position_est) {

	VectorXf UpdatedObservation(LandmarkDimensions);
	VectorXf Range(LandmarkDimensions);
	for (int i = 0; i < LandmarkDimensions; i++) {
		Range << landmark_position_est[i] - robot_pose_est[i];
	}
	float SquaredDistance = (float) Range.dot(Range.transpose());
	float EuclideanDistance = std::sqrt(SquaredDistance);
	
	float orientation = atan2(Range[1], Range[0]) - robot_pose_est[2];

	UpdatedObservation << EuclideanDistance, orientation;
	
	return UpdatedObservation;
}



void EKFSlam::BuildUpdateFunctionFor_G(PosePlus MeasuredPose, float translation, float rotation) {
	
	// Initialize each element in X as an Auto-Diff Object (Equivalent to a variable x)
	for (size_t i = 0; i < PoseDimensions; i++) {
		
		Xg[i] = AD<float>(0);
	}

	// Declare variables as Independent Variables. And Start Recording (A Gradient Tape Process).
		// Gradient Tape Process: Creates an Operation Sequence
		// Operation Sequence: All operations that depend on the elements of X are recorded on an active tape.
	Independent(Xg);

	// Set up your functions that will be Auto-Differentiated
	Yg[0] = MeasuredPose.RobotPose[0] - (PreviousPose[0] + (-1*(translation / rotation)) * CppAD::sin(Xg[2]) 
			+ (-1*(translation / rotation)) * CppAD::sin(Xg[2] + rotation * time_interval) );
	Yg[1] = MeasuredPose.RobotPose[1] - (PreviousPose[1] + (-1*(translation / rotation)) * CppAD::cos(Xg[2]) 
			+ (-1*(translation / rotation)) * CppAD::cos(Xg[2] + rotation * time_interval) );
	Yg[2] = MeasuredPose.RobotPose[2] - (PreviousPose[2] + rotation * time_interval);


	// Creates f: x -> y and stops tape recording
		// Performs the derivative calculations on the empty x variables.
	UpdateFunction = CppAD::ADFun<float>(Xg, Yg);
}	



void EKFSlam::BuildObservationFunctionFor_H(PosePlus MeasuredPose) {
	
	// Initialize each element in X as an Auto-Diff Object (Equivalent to a variable x)
	for (size_t i = 0; i < PoseDimensions; i++) {
		
		Xh[i] = AD<float>(0);
	}

	// Declare variables as Independent Variables. And Start Recording (A Gradient Tape Process).
		// Gradient Tape Process: Creates an Operation Sequence
		// Operation Sequence: All operations that depend on the elements of X are recorded on an active tape.
	Independent(Xh);

	// Set up your functions that will be Auto-Differentiated
	Yh[0] = MeasuredPose[0] - CppADD::cos(Xh[0]);
	Yh[1] = MeasuredPose[1] - ;
	Yh[2] = MeasuredPose[2] - ;
	Yh[3] = ;


	// Creates f: x -> y and stops tape recording
		// Performs the derivative calculations on the empty x variables.
	UpdateFunction = CppAD::ADFun<float>(Xh, Yh);
}	



void EKFSlam::Prediction(VectorXf current_pose, float trans_vel, float rot_vel) {

	// STEP 1: Update the State Vector. -----------------
	VectorXf updated_pose = UpdatePose(current_pose, trans_vel, rot_vel);
		// Map updated pose to entire state vector
	StateVector = StateVector + UpdateMappingFunction.transpose() * updated_pose;

	// STEP 2: Update Covariance Matrix -----------------
		
		// Take Jacobian of your Motion Model g()
		MatrixXf G = CalculateJacobian(UPDATE, StateVector);

		// Update the Covariance
		UpdateCovariance = G * UpdateCovariance * G.transpose() + ;// Need to add R (the uncertainty of motion)
}



void EKFSlam::Correction() {
	
	// STEP 1: Take the observation function h() which computes the expected observation.  -----------------


	// STEP 2: Compute Jacobian of Observation function h() - H -----------------
	MatrixXf H = CalculateJacobian(OBSERVATION, StateVector);
		// Map the low dimensional Jacobian back to Higher dim F
	HighDimension_H = H * ObservationMappingFunction;

	// STEP 3: Compute the Kalman Gain -----------------
	MatrixXf KalmanSegment = (H * UpdateCovariance * H.transpose()) + ObservationCovariance;
	KalmanGain = UpdateCovariance * H.transpose() * KalmanSegment.inverse(); 

	// STEP 4: Compute updated state & covariance -----------------
	StateVector = StateVector + KalmanGain * ; // Something
	UpdateCovariance = (Identity - (KalmanGain * H)) * UpdateCovariance;
}



MatrixXf EKFSlam::CalculateJacobian(FunctionType f_type, StateVec StateVector) {
	
	VectorXf robot_pose(PoseDimensions, 1);
	for (int i = 0; i < robot_pose.rows(); i++) {
		
		for (int j = 0; j < robot_pose.cols(); j++) {

			robot_pose << StateVector.Pose.RobotPose[i];
		}
	}


	// STEP 1: Set Up Update Function----------------------------------------------
	if (f_type == UPDATE) {
		
		BuildUpdateFunctionFor_G(robot_pose, 
				StateVector.Pose.RobotTranslation, 
				StateVector.Pose.RobotRotation);	
	}

	else if (f_type == OBSERVATION) {
		
		BuildObservationFunctionFor_H(robot_pose);	
	}
	
	// STEP 2: Compute the Jacobian of the Update Function ------------------------
	int rows = PoseDimensions;
	int cols = 2;
	int nonZeroElements = 2 * PoseDimensions;
	
	// Create vector of variables Jacobian will be calculated with respect to ( G(x) ).
	// Holds the value of the corresponding Independent Variable's index.
	// (e.g., 0 = X[0], 1 = X[1], etc.)
	std::vector<float> WithRespectTo(PoseDimensions);
	for (size_t i = 0; i < PoseDimensions; i++) {
		WithRespectTo[i] = (float) i;
	}
		
	// Compute the Jacobian***********
	if (f_type == UPDATE) {
		size_t n_color = UpdateFunction.Jacobian(WithRespectTo);
	}
	
	if (f_type == OBSERVATION) {
		size_t n_color = ObservationFunction.Jacobian(WithRespectTo);
	}

	MatrixXf Jac;
	
	// Return a Matrix
	return Jac;
}



void EKFSlam::BuildUpdateCovarianceAndStateVector(int pose_dim, int landmark_dim, int landmark_num, 
		VectorXf pose, std::vector<VectorXf> landmarks) {

	VectorXf state_vec;
	state_vec.Zero(1, pose_dim + (landmark_dim * landmark_num));
	
	for (int i = 0; i < pose_dim; i++) {
		state_vec[i] = pose[i];
	}

	int index_adjust = 0;
	for (int i = 0; i < landmark_num; i++) {

		for (int j = 0; j < landmark_dim; j++) {

			state_vec[pose_dim + (i + j + index_adjust)] = landmarks[i][j];
		}
		index_adjust++;
	}

	UpdateCovariance = state_vec * state_vec.transpose();
	StateVector = state_vec.transpose();
}



EKFSlam(StateVec initial_state, int pose_dim, int landmark_dim, 
		int landmark_num, MatrixXf obs_covariance) {

		// Set Dimensions
		PoseDimensions = pose_dim;
		LandmarkDimensions = landmark_dim;
		NumOfLandmarks = landmark_num;

		InitialState = initial_state;
		
		// Set up Covariance Matrices
		BuildUpdateCovarianceAndStateVector(PoseDimensions, LandmarkDimensions, NumOfLandmarks, 
				initial_state.RobotPose, initial_state.Map);
		ObservationCovariance = obs_covariance;

		// Set up Mapping Functions
		UpdateMappingFunction.Zero(PoseDimensions, PoseDimensions + (2 * NumOfLandmarks));
		ObservationMappingFunction.Zero(PoseDimensions + LandmarkDimensions, PoseDimensions + (2 * NumOfLandmarks));

		for (int i = 0; i < PoseDimensions; i++) { 
			UpdateMappingFunction[i][i] = 1;
			ObservationMappingFunction[i][i] = 1;
		}

		Identity = MatrixXf::Identity(); // Set Dimensions
}



void EKFSlam::Run() {
	
}





/*
 * 			TO-DO
 * 			-----
 *  - Finish
 *
 *  - Test Code
 *  
 *  - 
 *  */
