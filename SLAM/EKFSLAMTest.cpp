#include <gtest/gtest.h>
#include "EKFSLAM.hpp"


class EKFSlamTest : public ::testing::Test {

	protected:

		EKFSlamTest() {

			Eigen::VectorXf initial_pos(3);
			initial_pos << 0, 0, 0;
			int max_landmarks = 50;
			int pose_dim = 3;
			int landmark_dim = 2;


			// Map Setup
			std::vector<VectorXf> map;
			for (int i = 0; i < max_landmarks; i++) {

				// Just some junk landmarks
				VectorXf vec(2);
				vec << i, i + 1;
				map.push_back(vec);
			}

			// Setup Covariances FINISH!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			Eigen::MatrixXf obs_cov;
			Eigen::MatrixXf motion_cov;

			ekf = new EKFSlam(initial_pos, map, pose_dim, landmark_dim, max_landmarks, obs_cov, motion_cov);
		}
			
		//~EKFSlamTest() {}

		EKFSlam * ekf;
};



/**
 * @brief 
 * **/
TEST_F(EKFSlamTest, Run1) {

	ekf->Run();
}
