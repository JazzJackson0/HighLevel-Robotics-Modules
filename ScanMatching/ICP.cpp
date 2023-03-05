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



ICP::ICP(int pose_dim) {
	
	PoseDimension = pose_dim;

	std::vector<AD<float>> xs(PoseDimension);
	std::vector<AD<float>> ys(PoseDimension);
	X = xs;
	Y = ys;
}



void ICP::RunSVDAlign(PointCloud RefPointSet, PointCloud NewPointSet) {
	
	PointCloud AlignedPoints = NewPointSet; // Probably need a deep copy (maybe something to copy the vectors inside)
	VectorXf TrueCenterMass; // Initialize to 0
	VectorXf EstimatedCenterMass; // Initialize to 0
	VectorXf a; 
	VectorXf b; 
	MatrixXf H; // Initialize to 0
	float TrueWeightSum = 0.0;
	float EstWeightSum = 0.0;
	int cloud_size;

	if (RefPointSet.Points.size() > NewPointSet.Points.size()) {
		cloud_size = RefPointSet.Points.size();	
	}
	cloud_size = NewPointSet.Points.size();

	// Calculate Centers of Mass
	for (int i = 0; i < cloud_size; i++) {
		// Check that i doesnt go over point cloud size.
		TrueCenterMass += RefPointSet.Points[i]; // Multiply by Scalar
		TrueWeightSum += RefPointSet.Weights[i]; // Divide Sum by this 
		
		EstimatedCenterMass += NewPointSet.Points[i]; // Multiply by Scalar
		EstWeightSum += NewPointSet.Weights[i]; // Divide Sum by this 
	}

	// Calculate Cross-Covariance Matrix H
	for (int i = 0; i < cloud_size; i++) {
		
		a = (RefPointSet.Points[i] - TrueCenterMass);
		b = (NewPointSet.Points[i] - EstimatedCenterMass);
		
		H += a * b.transpose(); // Multiply by weight
	}
	
	// Compute Rotation Matrix (SVD Calculation)
	Eigen::JacobiSVD<MatrixXf> svd(H);
	MatrixXf RotationM = svd.matrixV() * svd.matrixU().transpose();

	// Compute Translation Vector
	VectorXf TranslationV = TrueCenterMass - RotationM * EstimatedCenterMass;

	// Translate & Rotate Cloud
	for (int i = 0; i < cloud_size; i++) {

		AlignedPoints.Points[i] = (RotationM * NewPointSet.Points[i]) + TranslationV;
		// Handle Weight Updates
	}
}



void ICP::RunSVD(PointCloud NewPointCloud) {
	
}



void ICP::RunLeastSquares(VectorXf x, PointCloud RefPointCloud, PointCloud NewPointCloud) {
	
	pair<PointCloud, PointCloud>  point_sets = Calculate_Correspondences(RefPointCloud, NewPointCloud);
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
 *  - Finish
 *
 *  - Test Code
 *  
 *  - 
 *  */








