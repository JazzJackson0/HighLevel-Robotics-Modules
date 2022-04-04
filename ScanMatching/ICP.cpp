#include "ICP.hpp"
#include <utility>


		void ICP::BuildErrorFunction(VectorXf MeasuredPose) {
			
			// Initialize each element in X as an Auto-Diff Object (Equivalent to a variable x)
			for (size_t i = 0; i < PoseDimension; i++) {
				
				X[i] = AD<float>(0);
			}

			// Declare variables as Independent Variables. And Start Recording (A Gradient Tape Process).
                // Gradient Tape Process: Creates an Operation Sequence
                // Operation Sequence: All operations that depend on the elements of X are recorded on an active tape.
			Independent(X);

			// Set up your functions that will be Auto-Differentiated
			//Y[0] = MeasuredPose[0] - CppADD::cos(X[0]) + Whatever...;
			//Y[1] = MeasuredPose[1] - ;
			//Y[2] = MeasuredPose[2] - ;
			//Y[3] = ;


            // Creates f: x -> y and stops tape recording
				// Performs the derivative calculations on the empty x variables.
			ErrorFunction = CppAD::ADFun<float>(X, Y);
		}	

		 

		MatrixXf ICP::CalculateJacobian(FunctionType f_type, std::vector<VectorXf> StateVector) {
			
			// STEP 1: Set Up Update Function----------------------------------------------
				
			BuildErrorFunction(StateVector.RobotPose);	
			
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
			size_t n_color = ErrorFunction.Jacobian(WithRespectTo);

			MatrixXf Jac;
			
			// Return a Matrix
			return Jac;
        }



		
		pair<PointCloud, PointCloud>  ICP::Calculate_Correspondences(PointCloud PointCloudA, PointCloud PointCloudB) {
			
			int a_size = PointCloudA.Points.size();
			int b_size = PointCloudB.Points.size();
			pair<PointCloud, PointCloud> Correspondences;
			PointCloud PointSetB;

			for (int i = 0; i < a_size; i++) {
				
				VectorXf point_a = PointCloudA.Points[i]; 
				float min_dist = std::numeric_limits<float>::max();
				int corresponding_indx = -1;

				for (int j = 0; j < b_size; j++) {
					
					VectorXf point_b = PointCloudB.Points[j]; 
					float dist = std::sqrt( (point_a * point_a) + (point_b * point_b) );

					if (dist < min_dist) {
						min_dist = dist;
						corresponding_indx = j;
					}
				}
				
				// Pull out the Point Set in B that corresponds directly with Point Cloud A
				PointSetB.Points.push_back(PointSetB.Points[j]);
				PointSetB.Weights.pop_back(PointSetB.Weights[j]);
			}

			Correspondences = std::make_pair(PointCloudA, PointSetB);
			return Correspondences;
		}
		


		ICP::ICP(int pose_dim) {
			
			PoseDimension = pose_dim;
		}



		void ICP::RunSVDAlign(PointCloud TruePointSet, PointCloud EstimatedPointSet) {
			
			PointCloud AlignedPoints = EstimatedPointSet; // Probably need a deep copy (maybe something to copy the vectors inside)
			VectorXf TrueCenterMass; // Initialize to 0
			VectorXf EstimatedCenterMass; // Initialize to 0
			VectorXf a; 
			VectorXf b; 
			MatrixXf H; // Initialize to 0
			float TrueWeightSum = 0.0;
			float EstWeightSum = 0.0;
			int cloud_size;

			if (TruePointSet.Points.size() > EstimatedPointSet.Points.size()) {
				cloud_size = TruePointSet.Points.size();	
			}
			cloud_size = EstimatedPointSet.Points.size();

			// Calculate Centers of Mass
			for (int i = 0; i < cloud_size; i++) {
				// Check that i doesnt go over point cloud size.
				TrueCenterMass += TruePointSet.Points[i]; // Multiply by Scalar
				TrueWeightSum += TruePointSet.Weights[i]; // Divide Sum by this 
				
				EstimatedCenterMass += EstimatedPointSet.Points[i]; // Multiply by Scalar
				EstWeightSum += EstimatedPointSet.Weights[i]; // Divide Sum by this 
			}

			// Calculate Cross-Covariance Matrix H
			for (int i = 0; i < cloud_size; i++) {
				
				a = (TruePointSet.Points[i] - TrueCenterMass);
				b = (EstimatedPointSet.Points[i] - EstimatedCenterMass);
				
				H += a * b.transpose(); // Multiply by weight
			}
			
			// Compute Rotation Matrix (SVD Calculation)
			Eigen::JacobiSVD<MatrixXf> svd(H);
			MatrixXf RotationM = svd.matrixV() * svd.matrixU().transpose();

			// Compute Translation Vector
			VectorXf TranslationV = TrueCenterMass - RotationM * EstimatedCenterMass;

			// Translate & Rotate Cloud
			for (int i = 0; i < cloud_size; i++) {

				AlignedPoints.Points[i] = (RotationM * EstimatedPointSet.Points[i]) + TranslationV;
				// Handle Weight Updates
			}
		}



		void ICP::RunSVD(PointCloud EstimatedPointCloud) {

		}



		void ICP::RunLeastSquares() {
		
			// A Very Rough Draft. I'm just assuming a while loop is the best form for this
			while () { // While not converged
					
				MatrixXf H_sum;
				VectorXf b_sum;
				
				// Compute sum of H and b over all N points.
				for (int n = 0; n < someethinglsakjdf; n++) {
					
					MatrixXf Jac = CalculateJacobian();
					MatrixXf H = Jac.transpose() * Jac;
					VectorXf b = Jac.transpose() * lksadjfasdf;

					H_sum += H;
					b_sum += b;
				}

				// Solve Linear System
				VectorXf x_uptate = H_sum.colPivHouseholderQr().solve(b_sum);	

				// Update Parameters


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








