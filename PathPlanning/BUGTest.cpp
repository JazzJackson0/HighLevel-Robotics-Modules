#include <gtest/gtest.h>
#include "BUG.hpp"


class BUGTest : public ::testing::Test {

	protected:

		BUGTest() {
			CellCoordinate pose_pos = std::make_pair(0, 0);
			Pose pose;
			pose.position = pose_pos;
			pose.orientation = 30.0;
			CellCoordinate goal = std::make_pair(50, 83);
			int forward_speed = 30;
			int turn_speed = 10;

			bug = new Bug(pose, goal, forward_speed, turn_speed);
		}
			
		//~BUGTest() {}

		Bug *bug;
};


/**
 * @brief
 *
 * **/
TEST_F(BUGTest, Bug0) {

	bug->Bug0();
}

/**
 * @brief
 *
 * **/
TEST_F(BUGTest, Bug1) {

	bug->Bug1();
}

/**
 * @brief
 *
 * **/
TEST_F(BUGTest, Bug2) {

	bug->Bug2();
}


/**
 * @brief
 *
 * **/
TEST_F(BUGTest, TangentBug) {

	bug->TangentBug();
}















