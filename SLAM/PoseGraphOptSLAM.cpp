#include "PoseGraphOptSLAM.hpp"
#include <Eigen/src/Core/util/Constants.h>
#include <bits/c++config.h>
#include <cppad/core/independent/independent.hpp>
#include <cppad/core/sparse_jac.hpp>
#include <cstddef>



std::vector<float> PoseGraphOptSLAM::Get_PoseData() {

}



void PoseGraphOptSLAM::BuildErrorFunction(VectorXf MeasuredPose, OdometryReadng odom) {
	
	// Initialize each element in X as an Auto-Diff Object (Equivalent to a variable x)
	for (size_t i = 0; i < PoseDimension; i++) {
		
		X[i] = AD<float>(0);
	}

	// Declare variables as Independent Variables. And Start Recording (A Gradient Tape Process).
		// Gradient Tape Process: Creates an Operation Sequence
		// Operation Sequence: All operations that depend on the elements of X are recorded on an active tape.
	Independent(X);

	// Set up your functions that will be Auto-Differentiated
	Y[0] = MeasuredPose[0] - (PreviousPose[0] + (-1*(trans_vel / rot_vel)) * CppAD::sin(X[2]) 
			+ (-1*(trans_vel / rot_vel)) * CppAD::sin(X[2] + rot_vel * time_interval) );
	Y[1] = MeasuredPose[1] - (PreviousPose[1] + (-1*(trans_vel / rot_vel)) * CppAD::cos(X[2]) 
			+ (-1*(trans_vel / rot_vel)) * CppAD::cos(X[2] + rot_vel * time_interval) );
	Y[2] = MeasuredPose[2] - (PreviousPose[2] + rot_vel * time_interval);

	// Creates f: x -> y and stops tape recording
		// Performs the derivative calculations on the empty x variables.
	ErrorFunction = CppAD::ADFun<float>(X, Y);
}	



std::vector<VectorXf> ICP(void) {
	

} 



std::vector<VectorXf> PoseGraphOptSLAM::Optimize(std::vector<VectorXf> StateVector) {

	std::vector<VectorXf> StateVectorUpdate;
	pair<Eigen::SparseMatrix<float, Eigen::RowMajor>, VectorXf> result;

	// While Nodes Not Converged
	while () { // Populate while loop

		// Build Linear System
		result = BuildLinearSystem(StateVector);

		// Solve the System
		LLT<MatrixXf> llt;
		llt.compute(result.first);
		StateVectorUpdate = llt.solve(result.second);

		// Update State Vector
		StateVector = StateVector + StateVectorUpdate; 

	}

	return StateVector;
}



pair<Eigen::SparseMatrix<float>, Eigen::RowMajor PoseGraphOptSLAM::BuildLinearSystem(
		std::vector<VectorXf> StateVector) {
	
	pair<Eigen::SparseMatrix<float, Eigen::RowMajor>, VectorXf> result;

	// STEP 1: Set Up Error Function----------------------------------------------
	float trans_vel = Get_PoseData()[3];
	float rot_vel = Get_PoseData()[4];
	BuildErrorFunction(StateVector.front(), trans_vel, rot_vel);	
	
	
	// STEP 2: Compute the Jacobian of the Error Function ------------------------
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
	Eigen::SparseMatrix<float, Eigen::RowMajor> Jac;
	sparse2eigen(SparseMatrix, Jac);

	// STEP 3: Linearize the Error Function -----------------
	VectorXf LinearizedErrorVector = ErrorVector + (Jac * variationAroundGuess);

	// STEP 4: Create the H Matrix & b vector -----------------
	// Compute Hessian			
	Eigen::SparseMatrix<float, Eigen::RowMajor> Hess = Jac.transpose() * (something * Jac);

	// Compute b
	VectorXf b = LinearizedErrorVector.transpose() * (lasdjfa * Jac);

	result.first = Hess;
	result.second = b;
	return result;
}



PoseGraphOptSLAM::PoseGraphOptSLAM(int max_nodes, int pose_dimension, int independent_val_num) {
	
	MaxStateVectorSize_N = max_nodes;
	MaxEdges = MaxStateVectorSize_N * 2;
	PoseDimension = pose_dimension;
	PreviousPose.setZero(pose_dimension);
}



void PoseGraphOptSLAM::Run() {
	
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
