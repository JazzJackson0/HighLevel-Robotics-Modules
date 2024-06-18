#include "ICP.hpp"

float ICP::Get_RootMeanSquaredError(PointCloud RefPointSet, PointCloud NewPointSet) {

	float rms_error = 0.f;
	int n = RefPointSet.points.size();

	for (int i = 0; i < n; i++) {

		rms_error += Get_EuclideanDistance(RefPointSet.points[i], NewPointSet.points[i]);
	}

	return sqrt(rms_error / n);
	// TODO: Squaring the data once here and once in 'Get_EuclideanDistance' might be a problem, look into it.
}

float ICP::Get_EuclideanDistance(VectorXf p, VectorXf q) {

	float dist = 0.f;
	for (int i = 0; i < p.rows(); i++) {

		dist += pow(p[i] - q[i], 2);
	}

	return sqrt(dist);
}

VectorXf ICP::Get_CenterOfMass(PointCloud p_cloud) {

	float total_weight = 0.0;
	VectorXf center_mass(PoseDimension);
	center_mass = VectorXf::Zero(PoseDimension);

	for (int i = 0; i < p_cloud.weights.size(); i++) {

		total_weight += p_cloud.weights[i];
	}

	for (int i = 0; i < p_cloud.points.size(); i++) {

		center_mass += (p_cloud.points[i] * p_cloud.weights[i]) / total_weight;
	}

	return center_mass;
}

PointCloud ICP::Update_PointCloud(PointCloud pointCloud, VectorXf x_increment) {

	// Create Rotation Matrix from angle increment.
	MatrixXf rotation(2, 2);
	rotation << std::cos(x_increment[2]), -1 * std::sin(x_increment[2]), 
		std::sin(x_increment[2]), std::cos(x_increment[2]);

	for (int i = 0; i < pointCloud.points.size(); i++) {
		VectorXf translated_point(PoseDimension);
		translated_point << (pointCloud.points[i][0] + x_increment[0]), (pointCloud.points[i][1] + x_increment[1]);
		pointCloud.points[i] = rotation * translated_point;
	}

	return pointCloud;
}

pair<PointCloud, PointCloud>  ICP::Calculate_Correspondences(PointCloud RefPointCloud, 
	PointCloud NewPointCloud) {
	
	int ref_size = RefPointCloud.points.size();
	//int new_size = NewPointCloud.points.size();
	PointCloud PointSet_New;
	struct Node* tree = kd_tree.build_tree(NewPointCloud.points);

	// Loop through all points in Reference Point Cloud
	for (int i = 0; i < ref_size; i++) {
		
		// Get a Point for comparison with New Cloud
		VectorXf ref_point = RefPointCloud.points[i]; 

		// ---------------------------Basic n^2 method not using kd tree---------------------------------------------------
		// float min_dist = std::numeric_limits<float>::max();
		// int corresponding_indx = -1;

		// // Loop through all points in New Point Cloud
		// for (int j = 0; j < new_size; j++) {
			
		// 	// Compare every point in New Point Cloud with Point from Reference.
		// 	VectorXf new_point = NewPointCloud.points[j]; 
		// 	float dist = std::sqrt( pow((ref_point[0] - new_point[0]), 2) + pow((ref_point[1] - new_point[1]), 2)
		// 		+ pow((ref_point[2] - new_point[2]), 2) );

		// 	if (dist < min_dist) {
		// 		min_dist = dist;
		// 		corresponding_indx = j;
		// 	}
		// }
		//--------------------------------------------------------------------------------------------------------------

		int corresponding_indx = kd_tree.get_nearest_neighbor(ref_point, tree, 0)->pos;
		
		// Pull out the Point Set in B that corresponds directly with Point Cloud A
		PointSet_New.points.push_back(NewPointCloud.points[corresponding_indx]);
		PointSet_New.weights.push_back(NewPointCloud.weights[corresponding_indx]);
	}

	return std::make_pair(RefPointCloud, PointSet_New);
}

VectorXf ICP::GetErrorVector(VectorXf x_param, VectorXf ReferencePoint, VectorXf NewPoint) {

	VectorXf error_vector(2);
	error_vector[0] = (cos(x_param[2]) * NewPoint[0]) - (sin(x_param[2]) * NewPoint[1]) + x_param[0] - ReferencePoint[0];
	error_vector[1] = (sin(x_param[2]) * NewPoint[0]) + (cos(x_param[2]) * NewPoint[1]) + x_param[1] - ReferencePoint[1];

	return error_vector;
}



void ICP::BuildErrorFunction(VectorXf ReferencePoint, VectorXf NewPoint) {
	
	// Initialize each element in X as an Auto-Diff Object (Equivalent to a variable x)
	for (size_t i = 0; i < ErrorDimension; i++) {
		
		X[i] = AD<float>(0);
	}

	// Declare variables as Independent Variables. And Start Recording (A Gradient Tape Process).
		// Gradient Tape Process: Creates an Operation Sequence
		// Operation Sequence: All operations that depend on the elements of X are recorded on an active tape.
	Independent(X);

	// Set up your functions that will be Auto-Differentiated
		// X = (t_x, t_y, angle) 
	Y[0] = (CppAD::cos(X[2]) * NewPoint[0]) - (CppAD::sin(X[2]) * NewPoint[1]) + X[0] - ReferencePoint[0];
	Y[1] = (CppAD::sin(X[2]) * NewPoint[0]) + (CppAD::cos(X[2]) * NewPoint[1]) + X[1] - ReferencePoint[1];

	// Creates f: x -> y and stops tape recording
		// Performs the derivative calculations on the empty x variables.
	ErrorFunction = CppAD::ADFun<float>(X, Y);
}	

	

MatrixXf ICP::CalculateJacobian(VectorXf ReferencePoint, VectorXf NewPoint, VectorXf x_update) {
	
	// STEP 1: Set Up Update Function----------------------------------------------	
	BuildErrorFunction(NewPoint, ReferencePoint);	
	
	// STEP 2: Compute the Jacobian of the Update Function ------------------------
	int rows = ErrorDimension;
	int cols = PoseDimension;
	
	// Create vector of variables Jacobian will be calculated with respect to ( J(x) ).
	// Holds the value of the corresponding Independent Variable's index.
	// (e.g., 0 = X[0], 1 = X[1], etc.)
	std::vector<float> WithRespectTo(ErrorDimension);
	for (size_t i = 0; i < ErrorDimension; i++) {
		WithRespectTo[i] = x_update[i];
	}
		
	// Compute the Jacobian***********
	std::vector<float> jac(PoseDimension * ErrorDimension);
	MatrixXf Jac;

	jac = ErrorFunction.Jacobian(WithRespectTo);

	Jac = MatrixXf::Zero(PoseDimension, ErrorDimension);
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


ICP::ICP() {
	// Defualt constructor
}


ICP::ICP(int pose_dim, int error_dim) : PoseDimension(pose_dim), ErrorDimension(error_dim) {

	std::vector<AD<float>> xs(ErrorDimension);
	std::vector<AD<float>> ys(PoseDimension);
	X = xs;
	Y = ys;
	min_convergence_thresh = 4; // Random value for now
	kd_tree = KDTree(PoseDimension);
}




RotationTranslation ICP::RunSVDAlign(PointCloud RefPointSet, PointCloud NewPointSet) {
	
	VectorXf TrueCenterMass; // Initialize to 0
	// VectorXf EstimatedCenterMass; // Initialize to 0
	VectorXf a; 
	VectorXf b; 
	MatrixXf H = MatrixXf::Zero(2, 2); 
	float TrueWeightSum = 0.0;
	int cloud_size;
	RotationTranslation transformation;
	transformation.weight = 0.0;

	if (RefPointSet.points.size() > NewPointSet.points.size()) {
		cloud_size = RefPointSet.points.size();	
	}

	else {
		cloud_size = NewPointSet.points.size();
	}
	
	// Calculate Centers of Mass & Weight Sums
	TrueCenterMass = Get_CenterOfMass(RefPointSet);
	transformation.center_mass = Get_CenterOfMass(NewPointSet);

	for (int i = 0; i < cloud_size; i++) {
		TrueWeightSum += RefPointSet.weights[i];
		transformation.weight += NewPointSet.weights[i];
	}

	// Calculate Cross-Covariance Matrix H
	for (int i = 0; i < cloud_size; i++) {
		
		a = (RefPointSet.points[i] - TrueCenterMass);
		b = (NewPointSet.points[i] - transformation.center_mass);
		
		H += a * b.transpose(); // Multiply by weight
	}
	
	// Compute Rotation Matrix (SVD Calculation)
	Eigen::JacobiSVD<MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV); // OR Eigen::ComputeFullU | Eigen::ComputeFullV
	transformation.rotation_matrix = svd.matrixV() * svd.matrixU().transpose(); // R = VU^T

	// Compute Translation Vector
	transformation.translation_vector = TrueCenterMass - transformation.rotation_matrix * transformation.center_mass;

	return transformation;
}



RotationTranslation ICP::RunICP_SVD(PointCloud RefPointCloud, PointCloud NewPointCloud) {

	RotationTranslation transformation;
	pair<PointCloud, PointCloud> point_sets = Calculate_Correspondences(RefPointCloud, NewPointCloud);
	PointCloud TransformedPointCloud = point_sets.second;

	float prev_error_norm = std::numeric_limits<float>::max();
	float error_norm = 1000000;
	float error_thresh = 0.5; // Bullshit number

	// Create Error Vector
	VectorXf error(TransformedPointCloud.points.size() * PoseDimension);
	error = VectorXf::Constant(TransformedPointCloud.points.size() * PoseDimension, std::numeric_limits<float>::max());
	// for (int i = 0; i < TransformedPointCloud.points.size() * PoseDimension; i++) {
	// 	error << std::numeric_limits<float>::max();
	// }

	while (error_norm < prev_error_norm && error_norm > error_thresh) {

		transformation = RunSVDAlign(point_sets.first, TransformedPointCloud);

		// Use the new Transformation to Align the point cloud: x_n = R(x_n - x_0) + y_0
		for (int i = 0; i < TransformedPointCloud.points.size(); i++)
		TransformedPointCloud.points[i] = (transformation.rotation_matrix * 
			(point_sets.second.points[i] - Get_CenterOfMass(point_sets.second))) + (Get_CenterOfMass(point_sets.first));
			

		// Calculate the new Error between the point clouds (Just the difference between the newly rotated point set and the reference point set)
		int cloud_idx = 0;
		for (int i = 0; i < TransformedPointCloud.points.size() * PoseDimension; i += PoseDimension) {

			for (int j = 0; j < PoseDimension; j++) {

				error(i + j) = (TransformedPointCloud.points[cloud_idx][j] - point_sets.first.points[cloud_idx][j]);
			}
			cloud_idx++;
		}
		
		// Calculate the New Error Norm (Error for the whole cloud)
		prev_error_norm = error_norm;
		for (int i = 0; i < TransformedPointCloud.points.size() * PoseDimension; i++) {

			error_norm += error(i) * error(i);
		}
		error_norm = sqrt(error_norm);
	}	

	return transformation;
}



VectorXf ICP::RunICP_LeastSquares(PointCloud RefPointCloud, PointCloud NewPointCloud) {
	
	pair<PointCloud, PointCloud> point_sets = Calculate_Correspondences(RefPointCloud, NewPointCloud);
	MatrixXf H_sum = MatrixXf::Zero(ErrorDimension, ErrorDimension);
	VectorXf b_sum = VectorXf::Zero(ErrorDimension);
	VectorXf x_update = VectorXf::Zero(ErrorDimension);
	int iterations = 0;

	// While Not Converged
	while (Get_RootMeanSquaredError(point_sets.first, point_sets.second) < min_convergence_thresh || iterations < 30) { 
			
		// Compute sum of H and b over all N points.
		for (int n = 0; n < point_sets.first.points.size() ; n++) {
			
			MatrixXf Jac = CalculateJacobian(point_sets.first.points[n], point_sets.second.points[n], x_update);
			MatrixXf H = Jac.transpose() * Jac;
			VectorXf b = Jac.transpose() * GetErrorVector(x_update, point_sets.first.points[n], point_sets.second.points[n]);

			H_sum += H;
			b_sum += b;
		}

		// Solve Linear System
		x_update += H_sum.colPivHouseholderQr().solve(b_sum);	

		// Update Parameters
		point_sets.second = Update_PointCloud(point_sets.second, x_update);
		iterations++;
	}

	return x_update;
}



/*
 * 			TO-DO
 * 			-----
 *  - Need a function to set the weight values for each point in a PointCloud
 * 
 *  - 
 *  
 *  - 
 *
 *  */








