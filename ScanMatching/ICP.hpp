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
typedef struct point_cloud PointCloud;
using namespace::CppAD;
using namespace::Eigen;

class ICP {
	
		int PoseDimension;
		int ErrorParameterNum = 3;
		std::vector<AD<float>> X; // X = (t_x, t_y, angle)
		std::vector<AD<float>> Y;
		ADFun<float> ErrorFunction;		

	private:

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
		 * @brief Builds the error function for the Non-Linear Least Squares ICP method.
		 *
		 * @param NewPoint 
		 * @param ReferencePoint 
		 *
		 * @return ** void
		 */
		void BuildErrorFunction(VectorXf NewPoint, VectorXf ReferencePoint); 
		 


		/**
         * @brief Creates and solves the Jacobian for a given function.
		 *
         * @param NewPoint 
		 * @param ReferencePoint 
		 * 
         * @return ** MatrixXf - Jacobian 
         */
		MatrixXf CalculateJacobian(VectorXf NewPoint, VectorXf ReferencePoint); 


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
		 * @return ** void
		 */
		void RunSVDAlign(PointCloud RefPointSet, PointCloud NewPointSet);



		/**
		 * @brief Run Point Cloud Registration with Unkown Data Association.
		 *
		 * @param NewPointCloud New Point Cloud
		 *
		 * @return ** void
		 */
		void RunSVD(PointCloud NewPointCloud);

		

		/**
		 * @brief Run Point Cloud Registration using a Non-Linear Least Squares apprroach.
		 *
		 * @param RefPointCloud Reference Point Cloud
		 * @param NewPointCloud New Point Cloud
		 * 
		 * @return ** void
		 */
		void RunLeastSquares(PointCloud RefPointCloud, PointCloud NewPointCloud);

};


struct point_cloud {

	std::vector<VectorXf> Points;
	std::vector<float> Weights;
};

