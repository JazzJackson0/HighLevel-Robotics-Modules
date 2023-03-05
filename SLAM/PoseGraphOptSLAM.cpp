#include "PoseGraphOptSLAM.hpp"

VectorXf PoseGraphOptSLAM::CreateStateVector(std::vector<VectorXf> poses) {

	VectorXf StateVector(MaxPoses);
	for (int i = 0; i < poses.size(); i++) {
		
		for (int j = 0; j < PoseDimensions; j++) {
			
			StateVector << poses[i][j];
		}
	}

	return StateVector;
}

VectorXf PoseGraphOptSLAM::GetErrorVector(VectorXf Pose_i, VectorXf Pose_j, VectorXf MeasuredTranslatedVector) {

	float x_delta = Pose_j[0] - Pose_i[0];
	float y_delta = Pose_j[0] - Pose_i[0];
	VectorXf error(3);
	
	error[0] = cos(MeasuredTranslatedVector[2]) * ((cos(Pose_i[2]) * x_delta + sin(Pose_i[2]) * y_delta) - MeasuredTranslatedVector[0]) 
		- sin(MeasuredTranslatedVector[2]) + ((sin(Pose_i[2]) * x_delta + cos(Pose_i[2]) * y_delta) - MeasuredTranslatedVector[1]);

	error[1] = -sin(MeasuredTranslatedVector[2]) * ((cos(Pose_i[2]) * x_delta + sin(Pose_i[2]) * y_delta) - MeasuredTranslatedVector[0]) 
		+ cos(MeasuredTranslatedVector[2]) + ((sin(Pose_i[2]) * x_delta + cos(Pose_i[2]) * y_delta) - MeasuredTranslatedVector[1]);

	error[2] = (Pose_j[2] - Pose_i[2]) - MeasuredTranslatedVector[2];

	return error;
}



void PoseGraphOptSLAM::BuildErrorFunction() {
	
	// Initialize each element in X as an Auto-Diff Object (Equivalent to a variable x)
	for (size_t i = 0; i < PoseDimensions; i++) {
		
		X[i] = AD<float>(0);
	}

	// Declare variables as Independent Variables. And Start Recording (A Gradient Tape Process).
		// Gradient Tape Process: Creates an Operation Sequence
		// Operation Sequence: All operations that depend on the elements of X are recorded on an active tape.
	Independent(X);
	
	// Set up your functions that will be Auto-Differentiated
	
	// X[0] = x j-i,   X[1] = x ij,   X[2] = y j-i,   X[3] = y ij,   X[4] = theta i,   X[5] = theta j,   X[6] = theta ij
	Y[0] = CppAD::cos(X[6]) * ((CppAD::cos(X[4]) * X[0] + CppAD::sin(X[4]) * X[2]) - X[1]) 
		- CppAD::sin(X[6]) + ((CppAD::sin(X[4]) * X[0] + CppAD::cos(X[4]) * X[2]) - X[3]);

	Y[1] = -CppAD::sin(X[6]) * ((CppAD::cos(X[4]) * X[0] + CppAD::sin(X[4]) * X[2]) - X[1]) 
		+ CppAD::cos(X[6]) + ((CppAD::sin(X[4]) * X[0] + CppAD::cos(X[4]) * X[2]) - X[3]);

	Y[2] = (X[5] - X[4]) - X[6];

	// Creates f: x -> y and stops tape recording
		// Performs the derivative calculations on the empty x variables.
	ErrorFunction = CppAD::ADFun<float>(X, Y);

}	



std::vector<VectorXf> PoseGraphOptSLAM::ICP(void) {
	
	return NULL;

} 



VectorXf PoseGraphOptSLAM::Optimize(std::vector<VectorXf> Poses, std::vector<PoseEdge> Edges, OdometryReadng odom) {

	VectorXf StateVector = CreateStateVector(Poses);
	VectorXf StateVectorUpdate;
	StateVectorUpdate = VectorXf::Ones(MaxPoses);
	pair<Eigen::SparseMatrix<float, Eigen::RowMajor>, VectorXf> total_result;
	pair<Eigen::SparseMatrix<float, Eigen::RowMajor>, VectorXf> temp_result;
	total_result.first = MatrixXf::Zero(MaxPoses, MaxPoses);
	total_result.second = VectorXf::Zero(MaxPoses);

	// While Nodes Not Converged
	while (StateVectorUpdate[0] > min_convergence_thresh) {


		// FIGURE OUT THE BEST WAY TO DO THIS!!
		for (int n = 0; n < Edges.size(); n++) { // Calculate & Sum H and b over every edge.

			VectorXf pose_i = Poses[Edges[n].PoseIndices.first];
			VectorXf pose_j = Poses[Edges[n].PoseIndices.second];
			VectorXf translated_vector = Edges[n].TransformationMatrix * pose_i;
			MatrixXf edge_covariance = Edges[n].NoiseInfoMatrix;
			
			// Build Linear System
			temp_result = BuildLinearSystem(pose_i, pose_j, translated_vector, edge_covariance, odom);

			total_result.first += temp_result.first; // Hessian
			total_result.second += temp_result.second; // Coefficient Vector b
		}
		

		// Solve the System x = H^-1 * b
		LLT<Eigen::MatrixXf> llt;
		llt.compute(total_result.first);
		StateVectorUpdate = llt.solve(total_result.second);

		// Update State Vector
		StateVector = StateVector + StateVectorUpdate; 

	}

	return StateVector;
}



pair<Eigen::SparseMatrix<float, Eigen::RowMajor>, VectorXf> PoseGraphOptSLAM::BuildLinearSystem(
		VectorXf pose_i, VectorXf pose_j, VectorXf MeasuredTranslatedVector, 
		MatrixXf edge_covariance, OdometryReadng odom) {
	
	pair<Eigen::SparseMatrix<float, Eigen::RowMajor>, VectorXf> result;

	// STEP 1: Set Up Error Function----------------------------------------------
	float trans_vel = Get_PoseData()[3];
	float rot_vel = Get_PoseData()[4];
	BuildErrorFunction();	
	
	
	// STEP 2: Compute the Jacobian of the Error Function ------------------------
	int rows = PoseDimensions;
	int cols = 2;
	int nonZeroElements = 2 * PoseDimensions;
	
	// Create vector of variables Jacobian will be calculated with respect to ( J(x) ).
	// Holds the value of the corresponding Independent Variable's index.
	// (e.g., 0 = X[0], 1 = X[1], etc.)
	ValueVector WithRespectTo(PoseDimensions);
	for (size_t i = 0; i < PoseDimensions; i++) {
		WithRespectTo[i] = (float) i;
	}

	// Set up Sparsity Pattern**********
	sparse_rc<SizeVector> SparsityPattern(rows, cols, nonZeroElements);
	for (size_t nonZeroIndex = 0; nonZeroIndex < nonZeroElements; nonZeroIndex++) {
		// Assign a position in the matrix for each non-zero element (indexed 0:nonZeroElements).
		// FIGURE OUT HOW TO CALCULATE THE CORRECT POSITIONS.
		size_t r = 0;
		size_t c;
		SparsityPattern.set(nonZeroIndex, r, c);
	}

	// Input the original Sparsity Pattern, apply any alterations then output Sparsity Pattern for J(x)
	bool TransposePattern = false;
	bool ConvertToDependency = false;
	bool BoolRepresentation = true;
	sparse_rc<SizeVector> JacobianSparsityPattern;
	ErrorFunction.for_jac_sparsity(SparsityPattern, TransposePattern, 
		ConvertToDependency, BoolRepresentation, JacobianSparsityPattern);

	// Set up Sparse Matrix***********
	// Specifies which elements of the Jacobian are computed
	sparse_rcv<SizeVector, ValueVector> SparseMatrix(JacobianSparsityPattern);
		
	// Compute the Sparse Jacobian***********
	CppAD::sparse_jac_work Work; // Stores Information used to reduce computation for future calls. 
	size_t ColorsPerGroup = 2; // The number of colors to undergo the [forward mode auto-diff process] At The Same Time.
	std::string ColoringAlgo = "cppad"; // Algorithm determining which columns will be computed during the same forward sweep.
	size_t n_color = ErrorFunction.sparse_jac_for(ColorsPerGroup, WithRespectTo, 
		SparseMatrix, JacobianSparsityPattern, ColoringAlgo, Work);
	
	// Convert to Eigen format
	Eigen::SparseMatrix<double, Eigen::RowMajor> Jac;
	CppAD::sparse2eigen(SparseMatrix, Jac);

	// STEP 3: Linearize the Error Function -----------------
	VectorXf LinearizedErrorVector = GetErrorVector(pose_i, pose_j, MeasuredTranslatedVector) + (Jac * VariationAroundGuess);

	// STEP 4: Create the H Matrix & b vector -----------------
	// Compute Hessian			
	Eigen::SparseMatrix<float, Eigen::RowMajor> Hess = Jac.transpose() * (edge_covariance * Jac);

	// Compute b
	VectorXf b = LinearizedErrorVector.transpose() * (edge_covariance * Jac);

	result.first = Hess;
	result.second = b;
	return result;
}



PoseGraphOptSLAM::PoseGraphOptSLAM(int max_nodes, int pose_dimension, int independent_val_num,
					int guess_variation) {
	
	MaxPoses = max_nodes;
	PoseDimensions = pose_dimension;
	PreviousPose.setZero(pose_dimension);
	VariationAroundGuess = guess_variation;

	std::vector<AD<float>> xs(PoseDimensions);
	std::vector<AD<float>> ys(PoseDimensions);  // Are these Dimensions Correct???????? NO THEY ARE NOT!!!
	X = xs;
	Y = ys;
}



void PoseGraphOptSLAM::Run() {
	
}


/*
 * 			TO-DO
 * 			-----
 *  - Finish
 *
 *  - Figure out how to loop over edges in Optimize function
 *  
 *  - Test Code
 *  */
