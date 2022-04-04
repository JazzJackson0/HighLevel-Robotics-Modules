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
		VecAD<float> X((size_t)PoseDimension);
		VecAD<float> Y(); // Assign correct size
		ADFun<float> ErrorFunction;		

	private:


		/**
		 * @brief 
		 *
		 * @param MeasuredPose
		 *
		 * @return ** void
		 */
		void BuildErrorFunction(VectorXf MeasuredPose); 
		 


		/**
         * @brief
		 *
         * 
         * @param StateVector The vector of poses to be optimized.
         * @return ** MatrixXf - Jacobian 
         */
		MatrixXf CalculateJacobian(FunctionType f_type, std::vector<VectorXf> StateVector); 



		/**
		 * @brief
		 *
		 * @param 
		 *
		 * @return **
		 */
		pair<PointCloud, PointCloud>  Calculate_Correspondences(PointCloud PointCloudA, PointCloud PointCloudB);



	public:
	

		/**
		 * @brief Initialize an Iterative Closest Point Algorithm Object
		 *
		 * @param pose_dim Poose Dimension
		 * **/	
		ICP(int pose_dim);



		/**
		 * @brief Run Point Cloud Registration with Known Data Association.
		 *
		 * @param 
		 *
		 * @return ** void
		 */
		void RunSVDAlign(PointCloud TruePointSet, PointCloud EstimatedPointSet);



		/**
		 * @brief Run Point Cloud Registration with Unkown Data Association.
		 *
		 * @param 
		 *
		 * @return ** void
		 */
		void RunSVD(PointCloud EstimatedPointCloud);

		

		/**
		 * @brief Run Point Cloud Registration using a Non-Linear Least Squares apprroach.
		 *
		 * @param
		 *
		 * @return ** void
		 */
		void RunLeastSquares();

};


struct point_cloud {

	vector<VectorXf> Points;
	vector<float> Weights;
};

