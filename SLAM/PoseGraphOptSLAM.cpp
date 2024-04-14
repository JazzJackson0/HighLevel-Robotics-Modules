#include "PoseGraphOptSLAM.hpp"

std::vector<float> PoseGraphOptSLAM::Get_PoseData() {

}


float PoseGraphOptSLAM::Calculate_Overlap(PointCloud landmarks_a, PointCloud landmarks_b) {

	float mean_ax = 0.0;
	float mean_ay = 0.0;
	float mean_bx = 0.0;
	float mean_by = 0.0;

	// Calculate the mean of each point cloud
	for (int i = 0; i < landmarks_a.Points.size(); i++) {
		mean_ax += landmarks_a.Points[i](0);
		mean_ay += landmarks_a.Points[i](1);
	}
	mean_ax /= landmarks_a.Points.size();
	mean_ay /= landmarks_a.Points.size();

 
	for (int i = 0; i < landmarks_b.Points.size(); i++) {
		mean_bx += landmarks_b.Points[i](0);
		mean_by += landmarks_b.Points[i](1);
	}
	mean_bx /= landmarks_b.Points.size();
	mean_by /= landmarks_b.Points.size();

	return (float) std::abs(sqrt((mean_ax - mean_bx)*(mean_ax - mean_bx) + (mean_ay - mean_by)*(mean_ay - mean_by)));
}


pair<VectorXf, std::vector<VectorXf>> PoseGraphOptSLAM::CreateStateVector() {

	VectorXf StateVector(MaxPoses);
	std::vector<VectorXf> rotation_axes;
	std::vector<Vertex<Pose, PoseEdge>> vertices = Pose_Graph.Get_Graph();

	// Loop through pose vertices
	for (int i = 0; i < vertices.size(); i++) {
		
		MatrixXf rotation = vertices[i].Data.TransformationMatrix.block(0, 0, 3, 3);
		VectorXf translation = vertices[i].Data.TransformationMatrix.block(0, 3, 1, 3);
		AngleAndAxis angle_axis = RotationMatrix_to_Angle(rotation);
		
		// Loop through pose dimensions (assuming pose dimension of 3 [x, y, theta])
		for (int j = 0; j < 2; j++)
			StateVector << translation(j);
		
		StateVector << angle_axis.first;
		rotation_axes.push_back(angle_axis.second);
	}

	return std::make_pair(StateVector, rotation_axes);
}

MatrixXf PoseGraphOptSLAM::VectorToTransformationMatrix(int x, int y, AngleAndAxis angle_axis) {

	MatrixXf rotation = Angle_to_RotationMatrix(angle_axis);
	VectorXf translation(3);
	translation << x, y, 0;
	MatrixXf transformation(rotation.rows(), rotation.cols() + 1);
	transformation << rotation, translation;

	return transformation;
}

VectorXf PoseGraphOptSLAM::GetErrorVector(VectorXf Pose_i, VectorXf Pose_j, VectorXf MeasuredTranslatedVector) {

	float x_delta = Pose_j[0] - Pose_i[0];
	float y_delta = Pose_j[0] - Pose_i[0];
	VectorXf error_vector(3);
	
	error_vector[0] = cos(MeasuredTranslatedVector[2]) * ((cos(Pose_i[2]) * x_delta + sin(Pose_i[2]) * y_delta) - MeasuredTranslatedVector[0]) 
		- sin(MeasuredTranslatedVector[2]) + ((sin(Pose_i[2]) * x_delta + cos(Pose_i[2]) * y_delta) - MeasuredTranslatedVector[1]);

	error_vector[1] = -sin(MeasuredTranslatedVector[2]) * ((cos(Pose_i[2]) * x_delta + sin(Pose_i[2]) * y_delta) - MeasuredTranslatedVector[0]) 
		+ cos(MeasuredTranslatedVector[2]) + ((sin(Pose_i[2]) * x_delta + cos(Pose_i[2]) * y_delta) - MeasuredTranslatedVector[1]);

	error_vector[2] = (Pose_j[2] - Pose_i[2]) - MeasuredTranslatedVector[2];

	return error_vector;
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




bool PoseGraphOptSLAM::FrontEnd(PointCloud current_landmarks) {
	
	Pose pose;
	PoseEdge edge;
	pose.Landmarks = current_landmarks;
	
	if (InitialScan)	
		PreviousLandmarks = current_landmarks;

	/* Check for level of overlap between landmark set of current & previous pose
		Add Pose & Edge to Graph if the amount of overlap is low enough*/
	if (Calculate_Overlap(PreviousLandmarks, current_landmarks) > OverlapTolerance || InitialScan) {

		InitialScan = false;
		ICP icp(PoseDimensions);
		RotationTranslation rot_trans = icp.RunSVD(PreviousLandmarks, current_landmarks); 
		MatrixXf R = rot_trans.rotation_matrix;
		VectorXf t = rot_trans.translation_vector;
		PreviousLandmarks = current_landmarks;
		
		// Turn R & t to a Transformation Matrix
		for (int i = 1; i <= R.cols() + 1; i++) {

			if (i <= R.cols())
				pose.TransformationMatrix.col(i) = R.col(1);
			else 
				pose.TransformationMatrix.col(i) = t;
		}

		// Add Pose to Graph
		bool connected = true;
		int v1 = -1;
		if (Pose_Graph.Get_NumOfVertices() == 0)
			connected = false;
		
		else {

			// Get Relative Transformation between 2 poses
			edge.TransformationMatrix = pose.TransformationMatrix * PreviousPose.Data.TransformationMatrix;
			int v1 = PreviousPose.VertexID;
			AllEdges.push_back(edge);
		}
			

		//edge.NoiseInfoMatrix = ; Calculate Noise Matrix??
		PreviousPose = Pose_Graph.Add_Vertex(pose, connected, edge);
		
		if (v1 >= 0) {
			int v2 = PreviousPose.VertexID;
			edge.PoseIndices = std::make_pair(v1, v2);
		}

		// Search Graph in given radius to find possible loop closure (Excluding the n most recently added poses)
		std::vector<Vertex<Pose, PoseEdge>> closure_candidates;
		std::vector<Vertex<Pose, PoseEdge>> vertices = Pose_Graph.Get_Graph();
		
		// No Loop Closure Happening
		if (vertices.size() <= NRecentPoses) {
			
			return false; 
		}

		// Loop Closure Process Start-------------------------------------------------------------------------------
		for (int i = 0; i < vertices.size() - NRecentPoses; i++) {

			// Calculate Distance
			if (sqrt(((vertices[i].Data.pose[0] - PreviousPose.Data.pose[0]) * 
				(vertices[i].Data.pose[0] - PreviousPose.Data.pose[0])) + 
				((vertices[i].Data.pose[1] - PreviousPose.Data.pose[1]) * 
				(vertices[i].Data.pose[1] - PreviousPose.Data.pose[1]))) 
				< ClosureDistance) {

				closure_candidates.push_back(vertices[i]);
			}
		}

		// Connect to the Closest out of those that are found in the radius.
		double closest = 1000000000000;
		int closest_vertex_idx = -1;
		for (int i = 0; i < closure_candidates.size(); i++) {

			if (sqrt(((closure_candidates[i].Data.pose[0] - PreviousPose.Data.pose[0]) * 
				(closure_candidates[i].Data.pose[0] - PreviousPose.Data.pose[0])) + 
				((closure_candidates[i].Data.pose[1] - PreviousPose.Data.pose[1]) * 
				(closure_candidates[i].Data.pose[1] - PreviousPose.Data.pose[1]))) 
				< closest) {
					
					closest_vertex_idx = closure_candidates[i].VertexID;
			}
		}

		// Loop Closure
		if (closest_vertex_idx >= 0) {

			PoseEdge closure_edge;
			closure_edge.TransformationMatrix = pose.TransformationMatrix * 
				Pose_Graph.Get_Vertex(closest_vertex_idx).TransformationMatrix;
			Pose_Graph.Add_Edge(Pose_Graph.Get_NumOfVertices(), closest_vertex_idx, closure_edge);
			AllEdges.push_back(closure_edge);
			return true;
		}
	}

	return false;
} 



void PoseGraphOptSLAM::Optimize(OdometryReadng odom) {
	Eigen::SparseMatrix<float, Eigen::RowMajor> SparseMat1(MaxPoses, MaxPoses);
	Eigen::SparseMatrix<float, Eigen::RowMajor> SparseMat2(MaxPoses, MaxPoses);

	pair<VectorXf, std::vector<VectorXf>> result = CreateStateVector();
	VectorXf StateVector = result.first;
	std::vector<VectorXf> rotation_axes = result.second; // Used for converting from State Vector back to Transformation matrices
	VectorXf StateVectorUpdate;
	StateVectorUpdate = VectorXf::Ones(MaxPoses);

	pair<Eigen::SparseMatrix<float, Eigen::RowMajor>, VectorXf> total_result;
	pair<Eigen::SparseMatrix<float, Eigen::RowMajor>, VectorXf> temp_result;
	total_result.first = SparseMat1;
	total_result.second = VectorXf::Zero(MaxPoses);

	// While Nodes Not Converged
	while (StateVectorUpdate[0] > min_convergence_thresh) {

		for (int n = 0; n < AllEdges.size(); n++) { // Calculate & Sum H and b over every edge.

			int edge_index_i = AllEdges[n].PoseIndices.first;
			int edge_index_j = AllEdges[n].PoseIndices.second;
			VectorXf pose_i = Pose_Graph.Get_Vertex(edge_index_i).pose;
			VectorXf pose_j = Pose_Graph.Get_Vertex(edge_index_j).pose;
			VectorXf translated_vector = Pose_Graph.Get_Edge(edge_index_i, edge_index_j).TransformationMatrix * pose_i;
			Eigen::SparseMatrix<float, Eigen::RowMajor> edge_covariance = Pose_Graph.Get_Edge(edge_index_i, edge_index_j).NoiseInfoMatrix;
			
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

	// Update all vectors in Graph with new Transformation Matrices. i.e. Convert from StateVector back to transformation matrices.
	int n = 0;
	std::vector<Vertex<Pose, PoseEdge>> vertices = Pose_Graph.Get_Graph();
	for (int i = 0; i < StateVector.size(); i = i + PoseDimensions) {
	
		vertices[n].Data.TransformationMatrix = VectorToTransformationMatrix(StateVector(i), StateVector(i+1), std::make_pair(StateVector(i+2), rotation_axes[n]));
		n++;
		Pose_Graph.Update_Data(n, vertices[n-1].Data);
	}


	// Update all corresponding edge transformation matrices: Loop through poses in graph
	for (int i = 0; i < vertices.size(); i++) {

		// Update all adjacent edges for pose i
		for (int j = 0; j < vertices[i].Adjacents.size(); j++) {

			PoseEdge edge = vertices[i].Adjacents[j].Weight;
			edge.TransformationMatrix = vertices[i].Data.TransformationMatrix 
				* Pose_Graph.Get_Vertex(vertices[i].Adjacents[j].AdjacentVertexID).TransformationMatrix;

			Pose_Graph.Update_Edge(i + 1, 
				vertices[i].Adjacents[j].AdjacentVertexID, edge);
		}
	}
}



pair<Eigen::SparseMatrix<float, Eigen::RowMajor>, VectorXf> PoseGraphOptSLAM::BuildLinearSystem(
		VectorXf pose_i, VectorXf pose_j, VectorXf MeasuredTranslatedVector, 
		Eigen::SparseMatrix<float, Eigen::RowMajor> edge_covariance, OdometryReadng odom) {
	
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
	Eigen::SparseMatrix<float, Eigen::RowMajor> Jac;
	CppAD::sparse2eigen(SparseMatrix, Jac);

	// STEP 3: Linearize the Error Function -----------------
	VectorXf LinearizedErrorVector = GetErrorVector(pose_i, pose_j, MeasuredTranslatedVector) + (Jac * VariationAroundGuess);

	// STEP 4: Create the H Matrix & b vector -----------------
	// Compute Hessian			
	Eigen::SparseMatrix<float, Eigen::RowMajor> omega_J = (edge_covariance * Jac);
	Eigen::SparseMatrix<float, Eigen::RowMajor> Hess(MaxPoses, MaxPoses);
	Hess = Jac.transpose() * omega_J;

	// Compute b
	VectorXf b = LinearizedErrorVector.transpose() * omega_J;

	result.first = Hess;
	result.second = b;
	return result;
}

//Public --------------------------------------------------------------------------------------------------------
PoseGraphOptSLAM::PoseGraphOptSLAM() {


}

PoseGraphOptSLAM::PoseGraphOptSLAM(int max_nodes, int pose_dimension, int independent_val_num,
					int guess_variation) {
	
	InitialScan = true;
	MaxPoses = max_nodes;
	PoseDimensions = pose_dimension;
	PreviousPose.Data.pose.setZero(pose_dimension);
	VariationAroundGuess = guess_variation;
	NRecentPoses = 5;
	OverlapTolerance = 1.0; // Just a random value for now!!

	std::vector<AD<float>> xs(PoseDimensions);
	std::vector<AD<float>> ys(PoseDimensions);  // Are these Dimensions Correct???????? NO THEY ARE NOT!!!
	X = xs;
	Y = ys;
}


void PoseGraphOptSLAM::FrontEndInit(int n_recent_poses, float closure_distance) {

	NRecentPoses = n_recent_poses;
	ClosureDistance = closure_distance;
}



void PoseGraphOptSLAM::Run(PointCloud current_landmarks, OdometryReadng odom) {
	
	if (FrontEnd(current_landmarks))
		Optimize(odom);
}


/*
 * 			TO-DO
 * 			-----
 *  - Finish
 *   
 *  - Test Code
 * 
 *  */
