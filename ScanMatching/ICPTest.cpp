#include <iostream>
#include <gtest/gtest.h>
#include "ICP.hpp"
 

class ICPTest : public ::testing::Test {

	protected:

		ICPTest() {
			
			float rand_val = 30.0;
			int pose_dimension = 3;

			// Setup 2 Point Clouds---------------

			// Old Point Cloud
			for (int i = 0; i < 30; i++) {
				
				VectorXf points(3);
				points << i, i + 1, i + 2;
				p_cloud_old.Points.push_back(points);
				p_cloud_old.Weights.push_back(i * rand_val);
				
				// Just making some random number
				rand_val++;
				if ((int)rand_val % 4 == 0) {
					rand_val /= 4;
				}
			}

			// New Point Cloud
			rand_val = 23.0;
			for (int i = 0; i < 30; i++) {
				
				VectorXf points(3);
				points << i, i + 1, i + 2;
				p_cloud_new.Points.push_back(points);
				p_cloud_new.Weights.push_back(i * rand_val);
				
				// Just making some random number
				rand_val++;
				if ((int)rand_val % 5 == 00) {
					rand_val /= 5;
				}
			}
			
			icp = new ICP(pose_dimension);
		}
			
		//~ICPTest() {}

		ICP *icp;
		PointCloud p_cloud_old;
		PointCloud p_cloud_new;
};


/**
 * brief
 *
 * **/
TEST_F(ICPTest, RunSVDAlign) {
	
	icp->RunSVDAlign(p_cloud_old, p_cloud_new);
}

/**
 * brief
 *
 * **/
TEST_F(ICPTest, RunSVD) {

	icp->RunSVD(p_cloud_old, p_cloud_new);
}

/**
 * brief
 *
 * **/
TEST_F(ICPTest, RunLeastSquares) {

	VectorXf error(3); // Look into what this is supposed to be.
	error << 1, 2, 3;

	icp->RunLeastSquares(error, p_cloud_old, p_cloud_new);
}




