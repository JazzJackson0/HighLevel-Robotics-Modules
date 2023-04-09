#include "EKFSLAM.hpp"

VectorXf EKFSlam::UpdatePose(VectorXf MeasuredPose, OdometryReadng odom) {
	
	VectorXf UpdatedPose(PoseDimensions);
	float trans = odom.RobotTranslation;
	float rot = odom.RobotRotation;
	UpdatedPose[0] = (MeasuredPose[0] + (-1*(trans / rot)) * sin(MeasuredPose[2]) 
			+ (-1*(trans / rot)) * sin(MeasuredPose[2] + rot * time_interval) );
	UpdatedPose[1] = (MeasuredPose[1] + (-1*(trans / rot)) * cos(MeasuredPose[2]) 
			+ (-1*(trans / rot)) * cos(MeasuredPose[2] + rot * time_interval) );
	UpdatedPose[2] = (MeasuredPose[2] + rot * time_interval);

	return UpdatedPose;
}


VectorXf EKFSlam::GetEstimatedScan(VectorXf robot_pose_est, 
	Point landmark_position_est) {

	VectorXf UpdatedObservation(LandmarkDimensions);
	VectorXf Range(LandmarkDimensions);
	
	Range << landmark_position_est.x - robot_pose_est[0];
	Range << landmark_position_est.y - robot_pose_est[1];

	// Calculate distance and angle between estimate and robot pose
	float SquaredDistance = (float) Range.dot(Range.transpose());
	float EuclideanDistance = std::sqrt(SquaredDistance);
	float orientation = atan2(Range[1], Range[0]) - robot_pose_est[2];

	UpdatedObservation << EuclideanDistance, orientation;
	
	return UpdatedObservation;
}



void EKFSlam::BuildUpdateFunctionFor_G(VectorXf MeasuredPose, OdometryReadng odom) {
	
	float trans = odom.RobotTranslation;
	float rot = odom.RobotRotation;
	std::vector<float> t(6);
	
	// Initialize each element in X as an Auto-Diff Object (Equivalent to a variable x)
	for (size_t i = 0; i < PoseDimensions; i++) {

		Xg[i] = AD<float>(0);
	}

	// Declare variables as Independent Variables. And Start Recording (A Gradient Tape Process).
		// Gradient Tape Process: Creates an Operation Sequence
		// Operation Sequence: All operations that depend on the elements of X are recorded on an active tape.
	Independent(Xg);

	// Set up your functions that will be Auto-Differentiated
	Yg[0] = MeasuredPose[0] - (PreviousPose[0] + (-1*(trans / rot)) * CppAD::sin(Xg[2]) 
			+ (-1*(trans / rot)) * CppAD::sin(Xg[2] + rot * time_interval) );
	Yg[1] = MeasuredPose[1] - (PreviousPose[1] + (-1*(trans / rot)) * CppAD::cos(Xg[2]) 
			+ (-1*(trans / rot)) * CppAD::cos(Xg[2] + rot * time_interval) );
	Yg[2] = MeasuredPose[2] - (PreviousPose[2] + rot * time_interval);


	// Creates f: x -> y and stops tape recording
		// Performs the derivative calculations on the empty x variables.
	UpdateFunction = CppAD::ADFun<float>(Xg, Yg);
}	



void EKFSlam::BuildObservationFunctionFor_H() {
	
	// Initialize each element in X as an Auto-Diff Object (Equivalent to a variable x)
	for (size_t i = 0; i < PoseDimensions + LandmarkDimensions; i++) {
		
		Xh[i] = AD<float>(0);
	}

	// Declare variables as Independent Variables. And Start Recording (A Gradient Tape Process).
		// Gradient Tape Process: Creates an Operation Sequence
		// Operation Sequence: All operations that depend on the elements of X are recorded on an active tape.
	Independent(Xh);

	// Set up your functions that will be Auto-Differentiated
		// X = (x, y, angle, landmark_x, landmark_y)
	Yh[0] = CppAD::sqrt( CppAD::pow(Xh[0], 2) + CppAD::pow(Xh[1], 2) );
	Yh[1] = CppAD::atan2(Xh[1], Xh[0]) - Xh[2];

	// Creates f: x -> y and stops tape recording
		// Performs the derivative calculations on the empty x variables.
	UpdateFunction = CppAD::ADFun<float>(Xh, Yh);
}	



MatrixXf EKFSlam::CalculateJacobian(FunctionType f_type, OdometryReadng odom) {
	
	int JacobianParameterDimension;

	VectorXf robot_pose;
	robot_pose = GetPoseFromStateVector();

	// STEP 1: Set Up Update Function----------------------------------------------
	if (f_type == UPDATE) {
		BuildUpdateFunctionFor_G(robot_pose, odom);
		JacobianParameterDimension = PoseDimensions;	
	}

	else if (f_type == OBSERVATION) {
		BuildObservationFunctionFor_H();
		JacobianParameterDimension = PoseDimensions + LandmarkDimensions;	
	}
	
	// STEP 2: Compute the Jacobian of the Update Function ------------------------
	int rows = PoseDimensions;
	int cols = 2;
	int nonZeroElements = 2 * PoseDimensions;
	
	// Create vector of variables Jacobian will be calculated with respect to ( G(x) ).
	// Holds the value of the corresponding Independent Variable's index.
	// (e.g., 0 = X[0], 1 = X[1], etc.)
	std::vector<float> WithRespectTo(JacobianParameterDimension);
	for (size_t i = 0; i < JacobianParameterDimension; i++) {
		WithRespectTo[i] = (float) i;
	}
		
	// Compute the Jacobian***********
	std::vector<float> jac;
	Eigen::MatrixXf Jac;

	if (f_type == UPDATE) {
		jac.resize(3 * JacobianParameterDimension);
		jac = UpdateFunction.Jacobian(WithRespectTo);
		Jac = MatrixXf::Zero(3, JacobianParameterDimension);
	}
	
	if (f_type == OBSERVATION) {
		jac.resize(2 * JacobianParameterDimension);
		jac = ObservationFunction.Jacobian(WithRespectTo);
		Jac = MatrixXf::Zero(2, JacobianParameterDimension);
	}

	for (int i = 0; i < jac.size(); i++) {

		Jac << jac[i];
	}
	
	// Return a Matrix
	return Jac;
}



void EKFSlam::BuildUpdateCovarianceAndStateVector(std::vector<VectorXf> landmarks) {

	VectorXf state_vec;
	state_vec.Zero(1, PoseDimensions + (LandmarkDimensions * NumOfLandmarks));
	
	for (int i = 0; i < PoseDimensions; i++) {
		state_vec[i] = InitialPosition[i];
	}

	int index_adjust = 0;
	for (int i = 0; i < NumOfLandmarks; i++) {

		for (int j = 0; j < LandmarkDimensions; j++) {

			state_vec[PoseDimensions + (i + j + index_adjust)] = landmarks[i][j];
		}
		index_adjust++;
	}

	UpdateCovariance = state_vec * state_vec.transpose();
	StateVector = state_vec.transpose();
}


VectorXf EKFSlam::GetPoseFromStateVector() {

	VectorXf currentPose;

	for (int i = 0; i < PoseDimensions; i++) {
		currentPose[i] = StateVector[i];
	}

	return currentPose;
}



void EKFSlam::Prediction(VectorXf current_pose, OdometryReadng odom) {
	
	// STEP 1: Update the State Vector. -----------------
	VectorXf updated_pose = UpdatePose(current_pose, odom);
		// Map updated pose to entire state vector
	StateVector = StateVector + UpdateMappingFunction.transpose() * updated_pose;

	// STEP 2: Update Covariance Matrix -----------------
		
		// Take Jacobian of your Motion Model g()
		Eigen::MatrixXf G = CalculateJacobian(UPDATE, odom);

		// Update the Covariance
		UpdateCovariance = G * UpdateCovariance * G.transpose() + MotionCovariance;
}



void EKFSlam::Correction(std::vector<Landmark> landmarks, OdometryReadng odom) {
	
	VectorXf current_pose = GetPoseFromStateVector();
	Landmark landmark_correspondence; // Populate this with landmark data from Correspondendce Matrix.
	
	// For all currently Obseerved Landmarks
	for (int i = 0; i < landmarks.size(); i++) {

		Landmark j = Correspondence[landmarks[i].id];
		if (j.err != 1) { 

			Landmark new_landmark_j;
			//new_landmark_j.position.x = current_pose[0] + (landmarks[i].range * cos(landmarks[i].bearing * current_pose[2]));
			//new_landmark_j.position.y = current_pose[1] + (landmarks[i].range * sin(landmarks[i].bearing * current_pose[2]));
			new_landmark_j.position = landmarks[i].position;

			// Add New Landmark to Correspondence Matrix
			Correspondence[landmarks[i].id] = new_landmark_j;
			landmark_correspondence = new_landmark_j;
		}

		// STEP 1: Take the observation function h() which computes the expected observation.  -----------------

			// Input current Landmark from Correspondence Matrix that can be seen by robot.
		VectorXf estimated_scan = GetEstimatedScan(current_pose, landmark_correspondence.position);

		// STEP 2: Compute Jacobian of Observation function h() - H -----------------
		Eigen::MatrixXf H = CalculateJacobian(OBSERVATION, odom);
			// Map the low dimensional Jacobian back to Higher dim F
		HighDimension_H = H * ObservationMappingFunction;

		// STEP 3: Compute the Kalman Gain -----------------
		Eigen::MatrixXf KalmanSegment = (H * UpdateCovariance * H.transpose()) + ObservationCovariance;
		KalmanGain = UpdateCovariance * H.transpose() * KalmanSegment.inverse(); 

		// STEP 4: Compute updated state & covariance -----------------
		VectorXf current_scan;
		current_scan << landmark_correspondence.range, landmark_correspondence.bearing; 

		StateVector = StateVector + KalmanGain * (current_scan - estimated_scan);
		UpdateCovariance = (Identity - (KalmanGain * H)) * UpdateCovariance;
	}
	
}



EKFSlam::EKFSlam(Eigen::VectorXf initial_position, std::vector<VectorXf> map, 
		int pose_dim, int landmark_dim, int max_landmark_num, 
		Eigen::MatrixXf obs_covariance, Eigen::MatrixXf motion_covariance) {

		// Set Dimensions
		PoseDimensions = pose_dim;
		LandmarkDimensions = landmark_dim;
		NumOfLandmarks = max_landmark_num;
		// Set Initial Position
		InitialPosition = initial_position;

		// Set Sizes of Domain & Range Space vectors
		std::vector<AD<float>> x_g(PoseDimensions);
		std::vector<AD<float>> y_g(PoseDimensions);
		std::vector<AD<float>> x_h(PoseDimensions);
		std::vector<AD<float>> y_h(LandmarkDimensions - PoseDimensions);
		Xg = x_g;
		Yg = y_g;
		Xh = x_h;
		Yh = y_h;
		
		// Set up Covariance Matrices & State Vector
		BuildUpdateCovarianceAndStateVector(map);
		ObservationCovariance = obs_covariance;
		MotionCovariance = motion_covariance;

		// Set up Mapping Functions
		UpdateMappingFunction.Zero(PoseDimensions, PoseDimensions + (2 * NumOfLandmarks));
		ObservationMappingFunction.Zero(PoseDimensions + LandmarkDimensions, 
			PoseDimensions + (2 * NumOfLandmarks));

		for (int i = 0; i < PoseDimensions; i++) { 
			UpdateMappingFunction(i, i) = 1.0;
			ObservationMappingFunction(i, i) = 1.0;
		}

		// Initialize Correspondence Matrix c with uninitialized landmarks
		for (int i = 0; i < 1000; i++) {
			Landmark lm;
			lm.err = 1;
			Correspondence[i] = lm;
		}

		Identity = MatrixXf::Identity(1, 2); // Set Dimensions THESE DIMENSIONS ARE TEMPORARY!!!!!!!!!!!!!!!!!!!!!!!
}



void EKFSlam::Run(std::vector<VectorXf> current_scan, VectorXf current_pose, OdometryReadng odom) {
	
	FeatureExtractor feature_extractor(0.5, 0.5, 4); // Random Garbage Values
	std::vector<Landmark> landmarks = feature_extractor.LandmarksFromScan(current_scan);

	Prediction(current_pose, odom);
	Correction(landmarks, odom);
}





/*
 * 			TO-DO
 * 			-----
 *
 *  - Test Code
 *  
 *  - 
 *  */
