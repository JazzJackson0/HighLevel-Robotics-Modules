#include "ICP.hpp"

PointCloud ICP::Update_PointCloud(PointCloud pointCloud, VectorXf x_increment) {

	for (int i = 0; i < pointCloud.Points.size(); i++) {
		pointCloud.Points[i][0] = x_increment[i];
		pointCloud.Points[i][1] = x_increment[i + 1];
		pointCloud.Points[i][2] = x_increment[i + 2];
	}

	return pointCloud;
}

pair<PointCloud, PointCloud>  ICP::Calculate_Correspondences(PointCloud RefPointCloud, 
	PointCloud NewPointCloud) {
	
	int ref_size = RefPointCloud.Points.size();
	int new_size = NewPointCloud.Points.size();
	pair<PointCloud, PointCloud> Correspondences;
	PointCloud PointSet_New;

	// Loop through all points in Reference Point Cloud
	for (int i = 0; i < ref_size; i++) {
		
		// Get a Point for comparison with New Cloud
		VectorXf ref_point = RefPointCloud.Points[i]; 
		float min_dist = std::numeric_limits<float>::max();
		int corresponding_indx = -1;

		// Loop through all points in New Point Cloud
		for (int j = 0; j < new_size; j++) {
			
			// Compare every point in New Point Cloud with Point from Reference.
			VectorXf new_point = NewPointCloud.Points[j]; 
			float dist = std::sqrt( pow((ref_point[0] - new_point[0]), 2) + pow((ref_point[1] - new_point[1]), 2)
				+ pow((ref_point[2] - new_point[2]), 2) );

			if (dist < min_dist) {
				min_dist = dist;
				corresponding_indx = j;
			}
		}
		
		// Pull out the Point Set in B that corresponds directly with Point Cloud A
		PointSet_New.Points.push_back(NewPointCloud.Points[corresponding_indx]);
		PointSet_New.Weights.push_back(NewPointCloud.Weights[corresponding_indx]);
	}

	Correspondences = std::make_pair(RefPointCloud, PointSet_New);
	return Correspondences;
}

VectorXf ICP::GetErrorVector(VectorXf x, VectorXf ReferencePoint, VectorXf NewPoint) {

	VectorXf error(2);
	error[0] = cos(x[2]) * NewPoint[0] - sin(x[2]) * NewPoint[1] + x[0] - ReferencePoint[0];
	error[1] = sin(x[2]) * NewPoint[0] + cos(x[2]) * NewPoint[1] + x[0] - ReferencePoint[1];

	return error;
}



void ICP::BuildErrorFunction(VectorXf ReferencePoint, VectorXf NewPoint) {
	
	// Initialize each element in X as an Auto-Diff Object (Equivalent to a variable x)
	for (size_t i = 0; i < ErrorParameterNum; i++) {
		
		X[i] = AD<float>(0);
	}

	// Declare variables as Independent Variables. And Start Recording (A Gradient Tape Process).
		// Gradient Tape Process: Creates an Operation Sequence
		// Operation Sequence: All operations that depend on the elements of X are recorded on an active tape.
	Independent(X);

	// Set up your functions that will be Auto-Differentiated
		// X = (t_x, t_y, angle) 
	Y[0] = CppAD::cos(X[2]) * NewPoint[0] - CppAD::sin(X[2]) * NewPoint[1] + X[0] - ReferencePoint[0];
	Y[1] = CppAD::sin(X[2]) * NewPoint[0] + CppAD::cos(X[2]) * NewPoint[1] + X[0] - ReferencePoint[1];

	// Creates f: x -> y and stops tape recording
		// Performs the derivative calculations on the empty x variables.
	ErrorFunction = CppAD::ADFun<float>(X, Y);
}	

	

MatrixXf ICP::CalculateJacobian(VectorXf ReferencePoint, VectorXf NewPoint) {
	
	// STEP 1: Set Up Update Function----------------------------------------------	
	BuildErrorFunction(NewPoint, ReferencePoint);	
	
	// STEP 2: Compute the Jacobian of the Update Function ------------------------
	int rows = PoseDimension;
	int cols = 2;
	int nonZeroElements = 2 * PoseDimension;
	
	// Create vector of variables Jacobian will be calculated with respect to ( J(x) ).
	// Holds the value of the corresponding Independent Variable's index.
	// (e.g., 0 = X[0], 1 = X[1], etc.)
	std::vector<float> WithRespectTo(PoseDimension);
	for (size_t i = 0; i < PoseDimension; i++) {
		WithRespectTo[i] = (float) i;
	}
		
	// Compute the Jacobian***********
	std::vector<float> jac(2 * PoseDimension);
	MatrixXf Jac;

	jac = ErrorFunction.Jacobian(WithRespectTo);

	Jac = MatrixXf::Zero(2, PoseDimension);
	for (int i = 0; i < jac.size(); i++) {

		Jac << jac[i];
	}

	// Return a Matrix
	return Jac;
}




VectorXf ICP::Get_CenterOfMass(PointCloud p_cloud) {

	float total_weight = 0.0;
	VectorXf center_mass(PoseDimension);
	for (int i = 0; i < PoseDimension; i++)
		center_mass << 0;


	for (int i = 0; i < p_cloud.Weights.size(); i++) {

		total_weight += p_cloud.Weights[i];
	}

	for (int i = 0; i < p_cloud.Points.size(); i++) {

		center_mass += (p_cloud.Points[i] * p_cloud.Weights[i]) / total_weight;
	}

	return center_mass;
}


ICP::ICP(int pose_dim) {
	
	PoseDimension = pose_dim;

	std::vector<AD<float>> xs(PoseDimension);
	std::vector<AD<float>> ys(PoseDimension);
	X = xs;
	Y = ys;
}




RotationTranslation ICP::RunSVDAlign(PointCloud RefPointSet, PointCloud NewPointSet) {
	
	PointCloud AlignedPoints = NewPointSet; // Probably need a deep copy (maybe something to copy the vectors inside)
	VectorXf TrueCenterMass; // Initialize to 0
	VectorXf EstimatedCenterMass; // Initialize to 0
	VectorXf a; 
	VectorXf b; 
	MatrixXf H; // Initialize to 0
	float TrueWeightSum = 0.0;
	int cloud_size;
	RotationTranslation transformation;
	transformation.weight = 0.0;

	if (RefPointSet.Points.size() > NewPointSet.Points.size()) {
		cloud_size = RefPointSet.Points.size();	
	}
	cloud_size = NewPointSet.Points.size();

	// Calculate Centers of Mass & Weight Sum
	TrueCenterMass = Get_CenterOfMass(RefPointSet);
	transformation.center_mass = Get_CenterOfMass(NewPointSet);
	for (int i = 0; i < cloud_size; i++) {
		TrueWeightSum += RefPointSet.Weights[i];
		transformation.weight += NewPointSet.Weights[i];
	}

	// Calculate Cross-Covariance Matrix H
	for (int i = 0; i < cloud_size; i++) {
		
		a = (RefPointSet.Points[i] - TrueCenterMass);
		b = (NewPointSet.Points[i] - transformation.center_mass);
		
		H += a * b.transpose(); // Multiply by weight
	}
	
	// Compute Rotation Matrix (SVD Calculation)
	Eigen::JacobiSVD<MatrixXf> svd(H);
	transformation.rotation_matrix = svd.matrixV() * svd.matrixU().transpose();

	// Compute Translation Vector
	transformation.translation_vector = TrueCenterMass - transformation.rotation_matrix * transformation.center_mass;

	// Translate & Rotate Cloud
	for (int i = 0; i < cloud_size; i++) {

		AlignedPoints.Points[i] = (transformation.rotation_matrix * NewPointSet.Points[i]) + transformation.translation_vector;
		// Handle Weight Updates
	}

	return transformation;
}



RotationTranslation ICP::RunSVD(PointCloud RefPointCloud, PointCloud NewPointCloud) {

	RotationTranslation transformation;
	pair<PointCloud, PointCloud> point_sets = Calculate_Correspondences(RefPointCloud, NewPointCloud);
	PointCloud TransformedPointCloud = point_sets.second;

	// Create Error Vector
	float prev_error_norm = 10000000000;
	float error_norm = 10000000;
	float error_thresh = 0.5; // Bullshit number

	VectorXf prev_error(TransformedPointCloud.Points.size());
	VectorXf error(TransformedPointCloud.Points.size());
	for (int i = 0; i < TransformedPointCloud.Points.size() * PoseDimension; i++) {
		prev_error << 10000000000;
		error << 10000000;
	}

	while (error_norm < prev_error_norm && error_norm > error_thresh) {

		transformation = RunSVDAlign(point_sets.first, TransformedPointCloud);

		// Apply Alignment
		for (int i = 0; i < TransformedPointCloud.Points.size(); i++)
		TransformedPointCloud.Points[i] = (transformation.rotation_matrix * 
			(point_sets.second.Points[i] - Get_CenterOfMass(point_sets.second))) + (Get_CenterOfMass(point_sets.first));

			
		// Recompute the Error Term (Just the difference between the newly rotated point set and the reference point set)
		for (int i = 0; i < TransformedPointCloud.Points.size() * PoseDimension; i += 3) {

			for (int j = 0; j < PoseDimension; j++) {

				error(i + j) = (TransformedPointCloud.Points[i][j] - point_sets.first.Points[i][j]);
			}
		}
		
		// Calculate the New Error Norm 
		prev_error_norm = error_norm;
		for (int i = 0; i < TransformedPointCloud.Points.size() * PoseDimension; i++) {

			error_norm += error(i) * error(i);
		}
		error_norm = sqrt(error_norm);
	}	

	return transformation;
}



void ICP::RunLeastSquares(VectorXf x, PointCloud RefPointCloud, PointCloud NewPointCloud) {
	
	pair<PointCloud, PointCloud> point_sets = Calculate_Correspondences(RefPointCloud, NewPointCloud);
	MatrixXf H_sum;
	VectorXf b_sum;
	VectorXf x_update = x;
	int i = 0;
	
	// While Not Converged
	while (x_update[0] > min_convergence_thresh) { 
			
		// Compute sum of H and b over all N points.
		for (int n = 0; n < point_sets.first.Points.size() ; n++) {
			
			MatrixXf Jac = CalculateJacobian(point_sets.first.Points[n], point_sets.second.Points[n]);
			MatrixXf H = Jac.transpose() * Jac;
			VectorXf b = Jac.transpose() * GetErrorVector(x_update, point_sets.first.Points[n], point_sets.second.Points[n]);

			H_sum += H;
			b_sum += b;
		}

		// Solve Linear System
		x_update = H_sum.colPivHouseholderQr().solve(b_sum);	

		// Update Parameters
		point_sets.second = Update_PointCloud(point_sets.second, x_update);
	}
}



/*
 * 			TO-DO
 * 			-----
 *  - Test Code
 *  
 *  - Need a function to set the weight values for each point in a PointCloud
 *
 *  */








