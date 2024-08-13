#include "EKFSLAM.hpp"

void EKFSlam::propagateFreeSpace() {

	int width = map_structure.dimension(1);
	int height = map_structure.dimension(0);
	VectorXi robot_index = map_builder.MapCoordinate_to_DataStructureIndex(PreviousPose);
    int x_robot = robot_index[0];
    int y_robot = robot_index[1];

    // Direction vectors for 8-connected neighbors
    int directions[8][2] = { {1, 0}, {0, 1}, {-1, 0}, {0, -1}, 
                             {1, 1}, {1, -1}, {-1, 1}, {-1, -1} };

    std::queue<VectorXi> q;
    q.push(robot_index);

    while (!q.empty()) {
        VectorXi index = q.front();
		int x = index[0];
		int y = index[1];
        q.pop();

        // Check all 8-connected neighbors
        for (auto& dir : directions) {
            int nx = x + dir[0];
            int ny = y + dir[1];

            // Check bounds
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                continue;
            }

            // Check if within the robot's view range
            if (std::sqrt(std::pow(nx - x_robot, 2) + std::pow(ny - y_robot, 2)) < VIEW_RANGE) {

                if (map_structure(ny, nx) == 0.5 && isVisible(nx, ny, x_robot, y_robot)) {
                    map_structure(ny, nx) = 0.0;
					VectorXi new_index(2);
					new_index << nx, ny;
                    q.push(new_index);
                }
            }
        }
    }
}

bool EKFSlam::isVisible(int x, int y, int x_robot, int y_robot) {
    
	if (map_structure(y, x) == 1.0) { return false;	}

    // Bresenham's line algorithm for line of sight checking
    int dx = std::abs(x - x_robot);
    int dy = std::abs(y - y_robot);
    int sx = (x_robot < x) ? 1 : -1;
    int sy = (y_robot < y) ? 1 : -1;
    int err = dx - dy;

    int x_curr = x_robot;
    int y_curr = y_robot;

    while (true) {
		// Clear line of sight
        if (x_curr == x && y_curr == y) { return true; }

		// Line of sight blocked by obstacle
        if (map_structure(y_curr, x_curr) == 1.0) { return false; }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x_curr += sx;
        }
        if (e2 < dx) {
            err += dx;
            y_curr += sy;
        }
    }
}

// Test this!!
Eigen::Tensor<float, 2> EKFSlam::UpdateMap() {

	VectorXi robot_index = map_builder.MapCoordinate_to_DataStructureIndex(PreviousPose);
	int width = map_structure.dimension(1);
	int height = map_structure.dimension(0);

	for (int i = 0; i < Correspondence.size(); i++) {

		for (int j = 0; j < Correspondence[i].points.size(); j++) {

			VectorXf point(2);
			point << Correspondence[i].points[i].x, Correspondence[i].points[i].y;
			VectorXi beam_index = map_builder.MapCoordinate_to_DataStructureIndex(point);

			if ((beam_index[0] >= 0 && beam_index[0] < width) && (beam_index[1] >= 0 && beam_index[1] < height)) {
				map_structure(beam_index[1], beam_index[0]) = 1.f;

				// Estimate Free Space: Bresenham's line algorithm for ray-casting
				int dx = std::abs(beam_index[0] - robot_index[0]);
				int dy = std::abs(beam_index[1] - robot_index[1]);
				int sx = (robot_index[0] < beam_index[0]) ? 1 : -1;
				int sy = (robot_index[1] < beam_index[1]) ? 1 : -1;
				int err = dx - dy;

				int x = robot_index[0];
				int y = robot_index[1];

				while (true) {

					if (map_structure(y, x) == 0.5) { map_structure(y, x) = 0.0; }

					if (map_structure(y, x) == 1.0) { break; }

					// Stop once end of line is reached
					if (x == beam_index[0] && y == beam_index[1]) { break; }

					// Climb the slope between robot and beam point
					int e2 = 2 * err;
					if (e2 > -dy) {
						err -= dy;
						x += sx;
					}
					if (e2 < dx) {
						err += dx;
						y += sy;
					}
				}
			}
		}
	}
	propagateFreeSpace();
	return map_structure;
}

void EKFSlam::Build_StateVector() {

	int LandmarkDim_X_N = (LandmarkDimensions * NumOfLandmarks);

	// Build State Vector
	StateVector = VectorXf::Zero(PoseDimensions + LandmarkDim_X_N);
	StateVector.block(0, 0, PoseDimensions, 1) = InitialPosition;

	if (LandmarkDim_X_N > 0) {

		int index_adjust = 0;
		for (int i = 0; i < NumOfLandmarks; i++) {

			for (int j = 0; j < LandmarkDimensions; j++) {

				StateVector[PoseDimensions + (i + j + index_adjust)] = Landmarks[i][j];
			}
			index_adjust++;
		}
	}
}


void EKFSlam::Build_Covariance() {

	int LandmarkDim_X_N = (LandmarkDimensions * NumOfLandmarks);

	// Build Covariance Matrix
	Covariance = MatrixXf::Zero((PoseDimensions + LandmarkDim_X_N), (PoseDimensions + LandmarkDim_X_N));
	DiagonalMatrix<float, Eigen::Dynamic, Eigen::Dynamic> landmark_covariance(PoseDimensions + LandmarkDim_X_N);
	landmark_covariance.diagonal().setConstant(0.01);
	DiagonalMatrix<float, Eigen::Dynamic, Eigen::Dynamic> pose_covariance(PoseDimensions);
	pose_covariance.diagonal().setConstant(0.001);
	Covariance = MatrixXf(landmark_covariance);
	Covariance.topLeftCorner(PoseDimensions, PoseDimensions) = MatrixXf(pose_covariance);
}


void EKFSlam::Build_NoiseCovariances() {

	int LandmarkDim_X_N = (LandmarkDimensions * NumOfLandmarks);

	// Build Covariance R
	ProcessNoiseCovariance_R = MatrixXf::Zero((PoseDimensions + LandmarkDim_X_N), (PoseDimensions + LandmarkDim_X_N));
	DiagonalMatrix<float, Eigen::Dynamic, Eigen::Dynamic> R(PoseDimensions);
	R.diagonal().setConstant(process_uncertainty_r);
	ProcessNoiseCovariance_R.topLeftCorner(PoseDimensions, PoseDimensions) = MatrixXf(R);

	// Build Covariance Q
	DiagonalMatrix<float, Eigen::Dynamic, Eigen::Dynamic> Q(LandmarkDimensions);
	Q.diagonal().setConstant(measurement_uncertainty_q);
	MeasurementCovariance_Q = Q;
}


void EKFSlam::Build_MappingFunctions() {

	int LandmarkDim_X_N = (LandmarkDimensions * NumOfLandmarks);

	PredictionMappingFunction_F = MatrixXf::Identity(PoseDimensions, PoseDimensions + LandmarkDim_X_N);
		
	ObservationMappingFunction_F = MatrixXf::Zero(PoseDimensions + LandmarkDimensions, PoseDimensions + LandmarkDim_X_N);
	Eigen::MatrixXf identity;
	identity.setIdentity(PoseDimensions, PoseDimensions);
	ObservationMappingFunction_F.topLeftCorner(PoseDimensions, PoseDimensions) = identity;
}


void EKFSlam::Build_Identity() {

	int LandmarkDim_X_N = (LandmarkDimensions * NumOfLandmarks);

	Identity = MatrixXf::Identity(PoseDimensions + LandmarkDim_X_N, 
			PoseDimensions + LandmarkDim_X_N);
}


int EKFSlam::UpdateMapAndResize(Landmark landmark) {
	
	// If State Vector has landmarks in it
	if (StateVector.rows() > PoseDimensions) {

		for (int i = PoseDimensions; i < StateVector.rows(); i += LandmarkDimensions) {

			float dist = std::sqrt(std::pow((StateVector[i] - landmark.position.x), 2) + std::pow((StateVector[i + 1] - landmark.position.y), 2));
			// If landmark point and map point are close enough to be virtually the same
			if (dist <= SimilarityMargin) {
				// Generic Formula for mapping State vector landmark to correct Correspondence index (Assumption: PoseDimensions >= LandmarkDimensions)
				return Correspondence[ ((i / LandmarkDimensions) - (PoseDimensions / LandmarkDimensions)) ].id;
			}
		}
	}

	// Resize State Vector-------------
	int original_size = StateVector.size();
	VectorXf landmark_vec(LandmarkDimensions);
	landmark_vec << landmark.position.x, landmark.position.y;
	VectorXf new_state_vector(original_size + LandmarkDimensions);
	new_state_vector << StateVector, landmark_vec;
	StateVector = new_state_vector;

	// Save landmark
	landmark.id = Correspondence.size();
	Correspondence.push_back(landmark);
	NumOfLandmarks++;

	// Resize matrices-------------
	int LandmarkDim_X_N = (LandmarkDimensions * NumOfLandmarks);

	// Resize Covariance (Sigma) 
	MatrixXf new_covariance = MatrixXf::Zero((PoseDimensions + LandmarkDim_X_N), (PoseDimensions + LandmarkDim_X_N));
	DiagonalMatrix<float, Eigen::Dynamic, Eigen::Dynamic> landmark_covariance(PoseDimensions + LandmarkDim_X_N);
	landmark_covariance.diagonal().setConstant(0.01);
	new_covariance = MatrixXf(landmark_covariance);
	new_covariance.topLeftCorner(Covariance.rows(), Covariance.cols()) = Covariance;
	Covariance = new_covariance;

	// Resize Covariance R & Q 
	Build_NoiseCovariances();

	// Resize Mapping Functions
	Build_MappingFunctions();

	// Resize Identity 
	Build_Identity();

	return landmark.id;
}


VectorXf EKFSlam::PredictPose_g(ControlCommand ctrl) {
	
	VectorXf PredictedPose(PoseDimensions);
	PredictedPose = VectorXf::Zero(3);
	float trans = ctrl.trans_vel;
	float rot = ctrl.rot_vel;
	PredictedPose[0] = (PreviousPose[0] + (-1*(trans / rot)) * std::sin(PreviousPose[2]) 
			+ (-1*(trans / rot)) * std::sin(PreviousPose[2] + rot * time_interval) );
	PredictedPose[1] = (PreviousPose[1] + (-1*(trans / rot)) * std::cos(PreviousPose[2]) 
			+ (-1*(trans / rot)) * std::cos(PreviousPose[2] + rot * time_interval) );
	PredictedPose[2] = (PreviousPose[2] + rot * time_interval);

	return PredictedPose;
}


VectorXf EKFSlam::GetEstimatedLandmark_h(VectorXf robot_pose_est, 
	Point landmark_position_est) {

	VectorXf landmark_estimate(LandmarkDimensions);
	VectorXf range_delta(LandmarkDimensions); // Range between estimated landmark & estimated pose
	range_delta << landmark_position_est.x - robot_pose_est[0], landmark_position_est.y - robot_pose_est[1];

	// Calculate distance and angle between estimate and robot pose
	float predicted_distance = std::sqrt((float) range_delta.dot(range_delta.transpose()));
	float predicted_orientation = std::atan2(range_delta[1], range_delta[0]) - robot_pose_est[2];
	
	landmark_estimate << predicted_distance, predicted_orientation;
	
	return landmark_estimate;
}

// g(x, y, theta)
void EKFSlam::BuildPredictionFunctionFor_G() {
	
	// Initialize each element in X as an Auto-Diff Object (Equivalent to a variable x)
	for (size_t i = 0; i < PoseDimensions; i++) {

		Xg[i] = AD<float>(0);
	}

	// Declare variables as Independent Variables. And Start Recording (A Gradient Tape Process).
		// Gradient Tape Process: Creates an Operation Sequence
		// Operation Sequence: All operations that depend on the elements of X are recorded on an active tape.
	Independent(Xg);

	float rot_vel = 1.f; // Actual value doesnt matter
	float trans_vel = 1.f; // Actual value doesnt matter
	float time = 1.f; // Actual value doesnt matter
	float fraction = (trans_vel/rot_vel);

	// Set up your functions that will be Auto-Differentiated
	// Differentiate w.r.t.: X[0] = x_prev, X[1] = y_prev, X[2] = theta_prev
	Yg[0] = (Xg[0] + (-1.f * fraction) * CppAD::sin(Xg[2]) + (fraction) * CppAD::sin(Xg[2] + (rot_vel * time)));
	Yg[1] = (Xg[1] + (fraction) * CppAD::cos(Xg[2]) + (-1.f * fraction) * CppAD::cos(Xg[2] + (rot_vel * time)) );
	Yg[2] = (fraction * time);


	// Creates f: x -> y and stops tape recording
		// Performs the derivative calculations on the empty x variables.
	PredictionFunction = CppAD::ADFun<float>(Xg, Yg);
}	


// h(x, y, theta, m_x, m_y)
void EKFSlam::BuildObservationFunctionFor_H() {
	
	// Initialize each element in X as an Auto-Diff Object (Equivalent to a variable x)
	for (size_t i = 0; i < (PoseDimensions + LandmarkDimensions); i++) {
		
		Xh[i] = AD<float>(0);
	}

	// Declare variables as Independent Variables. And Start Recording (A Gradient Tape Process).
		// Gradient Tape Process: Creates an Operation Sequence
		// Operation Sequence: All operations that depend on the elements of X are recorded on an active tape.
	Independent(Xh);

	AD<float> DiffX = Xh[3] - Xh[0];
	AD<float> DiffY = Xh[4] - Xh[1];
	
	// Supposedly temporary values.
	// TODO: Verify that this crap is indeed temporary, or what even 'temporary' means. Check the Jacobian output
	AD<float> supposedly_temporary1 = CppAD::sin(DiffY);
	AD<float> supposedly_temporary2 = CppAD::cos(DiffX);

	// Set up your functions that will be Auto-Differentiated
	// Differentiate w.r.t.: X[0] = pose_estimate_x, X[1] = pose_estimate_y, X[2] = pose_estimate_theta, X[3] = landmark_estimate_x, X[4] = landmark_estimate_y 
	Yh[0] = CppAD::sqrt( CppAD::pow(DiffX, 2) + CppAD::pow(DiffY, 2) );
	Yh[1] = CppAD::atan2(supposedly_temporary1, supposedly_temporary2) - Xh[2]; 
	//NOTE: Yh[1] = CppAD::atan2(DiffY, DiffX) - Xh[2]; // What the formula Should Actually be!!!! But CppAD::atan2() is bullshit

	// Creates f: x -> y and stops tape recording
		// Performs the derivative calculations on the empty x variables.
	ObservationFunction = CppAD::ADFun<float>(Xh, Yh);
}	



MatrixXf EKFSlam::CalculateJacobian(FunctionType f_type, int landmark_location) {
	
	int JacobianParameterDimension; //(i.e. w.r.t)

	VectorXf robot_pose = StateVector.block(0, 0, PoseDimensions, 1);
	VectorXf landmark;
	VectorXf g_inputs(PoseDimensions);
	VectorXf h_inputs(PoseDimensions + LandmarkDimensions);
	if (landmark_location >= 0) {
		landmark = StateVector.block(landmark_location, 0, LandmarkDimensions, 1);
		h_inputs << robot_pose, landmark;
	}

	g_inputs << robot_pose;


	// STEP 1: Set Up Prediction Function----------------------------------------------
	if (f_type == PREDICTION) {
		BuildPredictionFunctionFor_G();
		JacobianParameterDimension = PoseDimensions; // Num of independent variables in g to differentiate with respect to
	}

	else if (f_type == OBSERVATION) {
		BuildObservationFunctionFor_H();
		JacobianParameterDimension = (PoseDimensions + LandmarkDimensions);	// Num of independent variables in h to differentiate with respect to
	}
	
	// STEP 2: Compute the Jacobian of the Prediction Function ------------------------
	int rows = PoseDimensions;
	int cols = 2;
	int nonZeroElements = 2 * PoseDimensions;
	
	// Create vector of variables Jacobian will be calculated with respect to ( G(x) ).
	// Holds the value of the corresponding Independent Variable's index.
	// (e.g., 0 = X[0], 1 = X[1], etc.)
	std::vector<float> WithRespectTo(JacobianParameterDimension);
		
	// Compute the Jacobian***********
	std::vector<float> jac;
	Eigen::MatrixXf Jac;
	Jac = MatrixXf::Zero(1, 1); // Just dummy initialization

	if (f_type == PREDICTION) {

		for (size_t i = 0; i < JacobianParameterDimension; i++) {
			WithRespectTo[i] = g_inputs[i];
		}

		jac.resize(PoseDimensions * JacobianParameterDimension);
		jac = PredictionFunction.Jacobian(WithRespectTo);
		Jac = MatrixXf::Zero(PoseDimensions, JacobianParameterDimension);
	}
	
	if (f_type == OBSERVATION) {

		for (size_t i = 0; i < JacobianParameterDimension; i++) {
			WithRespectTo[i] = h_inputs[i];
		}

		jac.resize(LandmarkDimensions * JacobianParameterDimension);
		jac = ObservationFunction.Jacobian(WithRespectTo);
		Jac = MatrixXf::Zero(LandmarkDimensions, JacobianParameterDimension);
	}

	int k = 0;
	for (int i = 0; i < Jac.rows(); i++) {

		for (int j = 0; j < Jac.cols(); j++) {

			Jac(i, j) = jac[j + k];
		}

		k += Jac.cols();	
	}
	
	// Return a Matrix
	return Jac;
}



void EKFSlam::Prediction(VectorXf current_pose, ControlCommand ctrl) {
	
	// STEP 1: Update the State Vector. -----------------
	VectorXf predicted_pose = PredictPose_g(ctrl);
	PreviousPose = predicted_pose;
	// Map updated pose to entire state vector
	StateVector = StateVector + PredictionMappingFunction_F.transpose() * predicted_pose;
	
	// STEP 2: Update Covariance Matrix -----------------	
	// Take Jacobian of your Motion Model g()
	Eigen::MatrixXf G = CalculateJacobian(PREDICTION, -1);

	// Build High-Dimension G
	int LandmarkDim_X_N = (LandmarkDimensions * NumOfLandmarks);
	MatrixXf identity(LandmarkDim_X_N, LandmarkDim_X_N);
	identity.setIdentity();
	HighDimension_G = (PredictionMappingFunction_F.transpose() * (G * PredictionMappingFunction_F));
	HighDimension_G.block(PoseDimensions, PoseDimensions, LandmarkDim_X_N, LandmarkDim_X_N) = identity;

	// Update the Covariance
	Covariance = (HighDimension_G * (Covariance * HighDimension_G.transpose())) + ProcessNoiseCovariance_R;
}




void EKFSlam::Correction(std::vector<Landmark> landmarks) {
	
	VectorXf predicted_pose = PreviousPose;
	Landmark global_landmark;
	
	// For all currently Obseerved Landmarks
	for (int i = 0; i < landmarks.size(); i++) {

		global_landmark = Correspondence[UpdateMapAndResize(landmarks[i])];

		// STEP 1: Take the observation function h() which computes the expected observation.  -----------------

			// Input current Landmark from Correspondence Matrix that can be seen by robot.
		VectorXf estimated_landmark = GetEstimatedLandmark_h(predicted_pose, global_landmark.position);

		// STEP 2: Compute Jacobian of Observation function h() - H -----------------
		int landmark_location = PoseDimensions + (global_landmark.id * LandmarkDimensions); // Generic formula for getting correct column index from landmark id.
		Eigen::MatrixXf H = CalculateJacobian(OBSERVATION, landmark_location);

			// Update mapping function for landmark j	
		MatrixXf identity(LandmarkDimensions, LandmarkDimensions);
		identity.setIdentity();
		
		MatrixXf CurrentObservationMappingFunction_F = ObservationMappingFunction_F;
		CurrentObservationMappingFunction_F.block(PoseDimensions, landmark_location, LandmarkDimensions, LandmarkDimensions) = identity;

			// Map the low dimensional Jacobian back to Higher dim F
		HighDimension_H = H * CurrentObservationMappingFunction_F;

		// STEP 3: Compute the Kalman Gain -----------------
		Eigen::MatrixXf InnovationCovariance = (HighDimension_H * (Covariance * HighDimension_H.transpose())) + MeasurementCovariance_Q;
		KalmanGain = Covariance * HighDimension_H.transpose() * InnovationCovariance.inverse(); 

		// STEP 4: Compute updated state & covariance -----------------
		VectorXf current_landmark(LandmarkDimensions);
		current_landmark << global_landmark.range, global_landmark.bearing; 

		StateVector = StateVector + KalmanGain * (current_landmark - estimated_landmark);
		Covariance = (Identity - (KalmanGain * HighDimension_H)) * Covariance;
	}
	
}


EKFSlam::EKFSlam() {

	// Default constructor
}



EKFSlam::EKFSlam(int pose_dim, int landmark_dim) : PoseDimensions(pose_dim), LandmarkDimensions(landmark_dim) {

		//feature_extractor = FeatureExtractor (0.005, 0.5, 0.15, 9);
		feature_extractor = FeatureExtractor (0.5, 0.5, 0.15, 9);
		time_interval = 0.01;
		SimilarityMargin = 0.01; // m
		
		NumOfLandmarks = 0;
		PreviousPose = VectorXf::Zero(3);
		MapBuilder map_builder();
		
		// Set Sizes of Domain & Range Space vectors
		std::vector<AD<float>> x_g(PoseDimensions); // Number of variables to differentiate w.r.t
		std::vector<AD<float>> y_g(PoseDimensions);
		std::vector<AD<float>> x_h((PoseDimensions + LandmarkDimensions)); // Number of variables to differentiate w.r.t
		std::vector<AD<float>> y_h(LandmarkDimensions);
		Xg = x_g;
		Yg = y_g;
		Xh = x_h;
		Yh = y_h;

		// Set up Mapping Functions & Indentity
		Build_MappingFunctions();
		Build_Identity();
}


void EKFSlam::SetInitialState(Eigen::VectorXf initial_position, float _process_uncertainty_r, float _measurement_uncertainty_q) {
	

	InitialPosition = initial_position;
	process_uncertainty_r = _process_uncertainty_r; 
	measurement_uncertainty_q = _measurement_uncertainty_q;

	// Set up State Vector & Covariances
	Build_StateVector();
	Build_Covariance();
	Build_NoiseCovariances();

	// std::cout << "CURRENT MAP AFTER BUILD:" << std::endl;
	// std::cout << StateVector.transpose() << std::endl;
	// std::cout << "\n\n";
}


Eigen::Tensor<float, 2> EKFSlam::Run(PointCloud current_scan, VectorXf current_pose, ControlCommand ctrl) {
	
	std::vector<Landmark> landmarks = feature_extractor.LandmarksFromScan(current_scan, current_pose);

	Prediction(current_pose, ctrl);
	Correction(landmarks);

	// std::cout << "CURRENT MAP:" << std::endl;
	// std::cout << StateVector.transpose() << std::endl;
	// std::cout << "\n\n";

	return UpdateMap();
}


void EKFSlam::SetKnownLandmarks(std::vector<VectorXf> landmarks) {

	Landmarks = landmarks;
	NumOfLandmarks = landmarks.size();
	Build_StateVector();
	Build_Covariance();
}

void EKFSlam::Set_MapDimensions(int height, int width) {
	map_structure = Eigen::Tensor<float, 2>(height, width);
	map_structure.setConstant(0.5);
	map_builder.Update_2DMapDimensions(height, width);
}




/*
 * 			TO-DO
 * 			-----
 *
 *  - 
 *  
 *  - 
 *  */
