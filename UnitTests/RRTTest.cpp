#include <iostream>
#include <gtest/gtest.h>
#include </usr/include/eigen3/Eigen/Dense>
#include "/usr/include/eigen3/unsupported/Eigen/CXX11/Tensor"
#include "../PathPlanning/RRT.hpp"


class RRTTest : public ::testing::Test {

	protected:

		RRTTest() {

			float grid_storage[] = {1.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f, 1.f, 1.f,
								 	0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
								 	0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
								 	0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f,
								 	0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 1.f,
								 	0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
								 	0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f,
								 	0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f};

			Tensor<float, 2> grid(8, 10);
			auto &d = grid.dimensions();
			int rows = d[0];
			int cols = d[1];
			int k = 0;
			for (int i = 0; i < rows; i++) {

				for (int j = 0; j < cols; j++) {

					grid(i, j) = grid_storage[j + k];
				}

				k += cols;	
			}
			rrt = new RRT(grid);
		}
			
		~RRTTest() {
			delete rrt;
		}

		RRT *rrt;
};


/**
 * @brief
 *
 *
 * **/
TEST_F(RRTTest, RRT_Path1) {

	VectorXi start(2);
	VectorXi goal(2);
	start << 0, 0;
	goal << 9, 7;
	int max_connect_distance = 3;

	std::vector<VectorXi> path = rrt->RRT_Path(start, goal, max_connect_distance);

	for (int i = 0; i < path.size(); i++) {
		std::cout << path[i].transpose() << std::endl;
	}

	EXPECT_EQ(path[0][0], 0);
	EXPECT_EQ(path[0][1], 0);
}


/**
 * @brief
 *
 *
 * **/
TEST_F(RRTTest, RRT_Path2) {

	VectorXi start(2);
	VectorXi goal(2);
	start << 0, 1;
	goal << 9, 7;
	int max_connect_distance = 3;

	std::vector<VectorXi> path = rrt->RRT_Path(start, goal, max_connect_distance);

	for (int i = 0; i < path.size(); i++) {
		std::cout << path[i].transpose() << std::endl;
	}

	EXPECT_EQ(path[0][0], 0);
	EXPECT_EQ(path[0][1], 1);
	EXPECT_EQ(path[path.size() - 1][0], 9);
	EXPECT_EQ(path[path.size() - 1][1], 7);
}


/**
 * @brief
 *
 *
 * **/
TEST_F(RRTTest, RRT_Path3_LoadMap) {

	float grid_storage2[] = {1.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f, 1.f, 1.f, 1.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f, 1.f, 1.f,
							 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
							 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f,
							 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 1.f,
							 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
							 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f,
							 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
							 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 1.f,
							 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f, 1.f, 1.f,
							 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
							 1.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
							 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
							 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 1.f,
							 1.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f, 1.f};
	
	Tensor<float, 2> grid2(14, 20);
	auto &d2 = grid2.dimensions();
	int rows = d2[0];
	int cols = d2[1];
	int k = 0;
	for (int i = 0; i < rows; i++) {

		for (int j = 0; j < cols; j++) {

			grid2(i, j) = grid_storage2[j + k];
		}

		k += cols;	
	}
	
	VectorXi start(2);
	VectorXi goal(2);
	start << 19, 1;
	goal << 0, 12;
	int max_connect_distance = 3;
	
	rrt->Load_MAP(grid2);
	std::vector<VectorXi> path = rrt->RRT_Path(start, goal, max_connect_distance);

	for (int i = 0; i < path.size(); i++) {
		std::cout << path[i].transpose() << std::endl;
	}

	EXPECT_EQ(path[0][0], 19);
	EXPECT_EQ(path[0][1], 1);
	EXPECT_EQ(path[path.size() - 1][0], 0);
	EXPECT_EQ(path[path.size() - 1][1], 12);
}


/**
 * @brief
 *
 *
 * **/
TEST_F(RRTTest, RRTStar_Path) {
	
	VectorXi start(2);
	VectorXi goal(2);
	start << 0, 1;
	goal << 9, 7;
	int max_connect_distance = 3;
	int search_radius = 5;

	std::vector<VectorXi> path = rrt->RRTStar_Path(start, goal, max_connect_distance, search_radius);

	for (int i = 0; i < path.size(); i++) {
		std::cout << path[i].transpose() << std::endl;
	}

	EXPECT_EQ(path[0][0], 0);
	EXPECT_EQ(path[0][1], 1);
	EXPECT_EQ(path[path.size() - 1][0], 9);
	EXPECT_EQ(path[path.size() - 1][1], 7);
}




/**
 * @brief
 *
 *
 * **/
TEST_F(RRTTest, RRTStar_Path2_LoadMap) {
	
	float grid_storage2[] = {1.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f, 1.f, 1.f, 1.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f, 1.f, 1.f,
							 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
							 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f,
							 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 1.f,
							 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
							 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f,
							 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
							 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 1.f,
							 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f, 1.f, 1.f,
							 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
							 1.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
							 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
							 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 1.f,
							 1.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f, 1.f};
	
	Tensor<float, 2> grid2(14, 20);
	auto &d2 = grid2.dimensions();
	int rows = d2[0];
	int cols = d2[1];
	int k = 0;
	for (int i = 0; i < rows; i++) {

		for (int j = 0; j < cols; j++) {

			grid2(i, j) = grid_storage2[j + k];
		}

		k += cols;	
	}
	
	VectorXi start(2);
	VectorXi goal(2);
	start << 19, 1;
	goal << 0, 12;
	int max_connect_distance = 3;
	int search_radius = 5;

	rrt->Load_MAP(grid2);
	std::vector<VectorXi> path = rrt->RRTStar_Path(start, goal, max_connect_distance, search_radius);

	for (int i = 0; i < path.size(); i++) {
		std::cout << path[i].transpose() << std::endl;
	}

	EXPECT_EQ(path[0][0], 19);
	EXPECT_EQ(path[0][1], 1);
	EXPECT_EQ(path[path.size() - 1][0], 0);
	EXPECT_EQ(path[path.size() - 1][1], 12);
}

