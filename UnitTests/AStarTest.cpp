#include <iostream>
#include <vector>
#include <gtest/gtest.h>
#include </usr/include/eigen3/Eigen/Dense>
#include "/usr/include/eigen3/unsupported/Eigen/CXX11/Tensor"
#include "../PathPlanning/AStar.hpp"



class AStarTest : public ::testing::Test {

	protected:

		AStarTest() {

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

			astar = new A_Star(grid);
		}
			
		//~AStarTest() {}

		A_Star *astar;
		A_Star *astar2;
};


/**
 *	@brief 
 *
 * **/
TEST_F(AStarTest, Path) {
	
	VectorXi start(2);
	VectorXi goal(2);
	start << 0, 1;
	goal << 9, 7;
	std::vector<VectorXi> path = astar->Path(start, goal);

	EXPECT_EQ(path[0][0], 0);
	EXPECT_EQ(path[0][1], 1);

	EXPECT_EQ(path[1][0], 1);
	EXPECT_EQ(path[1][1], 2);

	EXPECT_EQ(path[2][0], 2);
	EXPECT_EQ(path[2][1], 2);

	EXPECT_EQ(path[3][0], 3);
	EXPECT_EQ(path[3][1], 3);

	EXPECT_EQ(path[4][0], 4);
	EXPECT_EQ(path[4][1], 3);

	EXPECT_EQ(path[5][0], 5);
	EXPECT_EQ(path[5][1], 3);

	EXPECT_EQ(path[6][0], 6);
	EXPECT_EQ(path[6][1], 3);

	EXPECT_EQ(path[7][0], 7);
	EXPECT_EQ(path[7][1], 4);

	EXPECT_EQ(path[8][0], 8);
	EXPECT_EQ(path[8][1], 5);

	EXPECT_EQ(path[9][0], 9);
	EXPECT_EQ(path[9][1], 6);

	EXPECT_EQ(path[10][0], 9);
	EXPECT_EQ(path[10][1], 7);
}


/**
 *	@brief 
 *
 * **/
TEST_F(AStarTest, Path2) {
	
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
	astar->Load_MAP(grid2);
	std::vector<VectorXi> path = astar->Path(start, goal);


	EXPECT_EQ(path[0][0], 19);
	EXPECT_EQ(path[0][1], 1);

	EXPECT_EQ(path[19][0], 0);
	EXPECT_EQ(path[19][1], 12);
}
