#include <iostream>
#include <gtest/gtest.h>
#include "PoseGraphOptSLAM.hpp"

class PoseGraphOptSLAMTest : public ::testing::Test {

	protected:

		PoseGraphOptSLAMTest() {
			
			int max_nodes = 100;
			int pose_dimension = 3;
			int independent_val = 10; // ???? I Don't remember what this is & its not used in the code, look at notes.
			int guess_variation = 1;

			pose_graph = new PoseGraphOptSLAM(max_nodes, pose_dimension, independent_val, guess_variation);
		}
			
		//~PoseGraphOptSLAMTest() {}

		PoseGraphOptSLAM *pose_graph;
};


/**
 * @brief 
 *
 * **/
TEST_F(PoseGraphOptSLAMTest, Run1) {

	PointCloud current_landmarks;
	OdometryReadng odom;
	int n_recent_poses = 5;
	float closure_distance = 1.0;
	
	pose_graph->FrontEndInit(n_recent_poses, closure_distance);
	pose_graph->Run(current_landmarks, odom);
}



