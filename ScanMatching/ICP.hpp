#include <iostream>
#include <vector>
#include <cmath>
#include <utility>

#include </usr/include/eigen3/Eigen/Dense>
#include </usr/include/eigen3/Eigen/SVD>
#include </usr/include/eigen3/Eigen/QR>

#include <cppad/cppad.hpp>

using std::vector;
using std::pair;
using Eigen::VectorXf;
using Eigen::MatrixXf;
using namespace::CppAD;
using namespace::Eigen;

struct PointCloud {
	std::vector<VectorXf> Points;
	std::vector<float> Weights;
};

struct RotationTranslation {
	MatrixXf rotation_matrix;
	VectorXf translation_vector;
	VectorXf center_mass;
	float weight;
};

class ICP {
	
		int PoseDimension;
		int ErrorParameterNum = 3;
		std::vector<AD<float>> X; // X = (t_x, t_y, angle)
		std::vector<AD<float>> Y;
		ADFun<float> ErrorFunction;
		float min_convergence_thresh;		

	private:

		/**
		 * @brief Updates a given Point Cloud with the incremental result from the solution
		 * 			to a Linear System.
		 * 
		 * @param PointCloud The Point Cloud to update
		 * @param x_increment The increment to add to the Point Cloud.
		 * @return ** PointCloud  Updated Point Cloud
		 */
		PointCloud Update_PointCloud(PointCloud PointCloud, VectorXf x_increment);
		
		/**
		 * @brief Calculates the correspondences between two point clouds and returns subsets
		 * 			of each cloud that map to each other (n-to-n).
		 *
		 * @param RefPointCloud - Reference Point Cloud
		 * @param NewPointCloud - New Point Cloud
		 *
		 * @return ** pair<PointCloud, PointCloud> - (Reference Point Set, New Point Set)
		 */
		pair<PointCloud, PointCloud>  Calculate_Correspondences(PointCloud RefPointCloud, 
			PointCloud NewPointCloud);

		/**
		 * @brief Get the Error between the new and reference point
		 * 
		 * @param x Error function parameters (x, y, theta)
		 * @param ReferencePoint Point n from Reference Point Set
		 * @param NewPoint Point n from New Point Set
		 * @return ** VectorXf 
		 */
		VectorXf GetErrorVector(VectorXf x, VectorXf ReferencePoint, VectorXf NewPoint);

		/**
		 * @brief Builds the error function for the Non-Linear Least Squares ICP method.
		 *
		 * @param ReferencePoint Point n from Reference Point Set
		 * @param NewPoint Point n from New Point Set
		 *
		 * @return ** void
		 */
		void BuildErrorFunction(VectorXf ReferencePoint, VectorXf NewPoint); 
		 


		/**
         * @brief Creates and solves the Jacobian for a given function.
		 *
		 * @param ReferencePoint Point n from Reference Point Set
		 * @param NewPoint Point n from New Point Set
		 * 
         * @return ** MatrixXf - Jacobian 
         */
		MatrixXf CalculateJacobian(VectorXf ReferencePoint, VectorXf NewPoint); 


		/**
		 * @brief Calculate the Center of Mass for a given Point Cloud
		 * 
		 * @param p_cloud The cloud you calculate the center of mass of
		 * @return VectorXf Center of Mass
		 */
		VectorXf Get_CenterOfMass(PointCloud p_cloud);


	public:
	

		/**
		 * @brief Initialize an Iterative Closest Point Algorithm Object.
		 *
		 * @param pose_dim Pose Dimension
		 * **/	
		ICP(int pose_dim);



		/**
		 * @brief Run Point Cloud Registration with Known Data Association.
		 *
		 * @param RefPointSet Reference Point Set
		 * @param NewPointSet New Point Set
		 *
		 * @return ** RotationTranslation
		 */
		RotationTranslation RunSVDAlign(PointCloud RefPointSet, PointCloud NewPointSet);



		/**
		 * @brief Run Point Cloud Registration with Unkown Data Association.
		 *
		 * @param RefPointCloud New Point Cloud
		 * @param NewPointCloud New Point Cloud
		 *
		 * @return ** RotationTranslation
		 */
		RotationTranslation RunSVD(PointCloud RefPointCloud, PointCloud NewPointCloud);

		

		/**
		 * @brief Run Point Cloud Registration using a Non-Linear Least Squares apprroach.
		 *
		 * @param x Error function parameters (x, y, theta)
		 * @param RefPointCloud Reference Point Cloud
		 * @param NewPointCloud New Point Cloud
		 * 
		 * @return ** void
		 */
		void RunLeastSquares(VectorXf x, PointCloud RefPointCloud, PointCloud NewPointCloud);
};




