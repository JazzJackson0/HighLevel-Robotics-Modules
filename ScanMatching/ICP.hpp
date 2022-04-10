#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/QR>
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
		VecAD<float> X((size_t)PoseDimension); // X = (t_x, t_y, angle)
		VecAD<float> Y(); // Assign correct size
		ADFun<float> ErrorFunction;		

	private:


		/**
		 * @brief Builds the error function for the Non-Linear Least Squares ICP method.
		 *
		 * @param MeasuredPose
		 *
		 * @return ** void
		 */
		void BuildErrorFunction(VectorXf MeasuredPose); 
		 


		/**
         * @brief Creates and solves the Jacobian for a given function.
		 *
         * @param StateVector The vector of poses to be optimized.
         * @return ** MatrixXf - Jacobian 
         */
		MatrixXf CalculateJacobian(FunctionType f_type, std::vector<VectorXf> StateVector); 



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
		 * @return ** void
		 */
		void RunLeastSquares();

};


struct point_cloud {

	vector<VectorXf> Points;
	vector<float> Weights;
};

