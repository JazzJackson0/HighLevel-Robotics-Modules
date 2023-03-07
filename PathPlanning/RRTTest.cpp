#include <gtest/gtest.h>
#include "RRT.hpp"


class RRTTest : public ::testing::Test {

	protected:

		RRTTest() {

			int grid_width = 10;
			int grid_height = 8;
			int pre_grid[grid_height][grid_width] = {{1, 1, 1, 0, 1, 1, 0, 0, 1, 1},
													{0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
													{0, 0, 1, 0, 0, 0, 0, 1, 0, 0},
													{0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
													{0, 0, 0, 0, 1, 0, 0, 1, 0, 0},
													{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
													{0, 1, 0, 0, 0, 0, 0, 1, 1, 0},
													{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
			int* grid_rows[grid_height] = {pre_grid[0], pre_grid[1], pre_grid[2], pre_grid[3], pre_grid[4], pre_grid[5], pre_grid[6], pre_grid[7]};
			int** grid = grid_rows;

			rrt = new RRT(grid, grid_width, grid_height);
		}
			
		//~RRTTest() {}

		RRT *rrt;
};


/**
 * @brief
 *
 *
 * **/
TEST_F(RRTTest, Run_RRT) {

	Node start_node;
	Node goal_node;
	start_node.x = 0;
	start_node.y = 0;
	start_node.DistanceFromStart = 0;
	goal_node.x = 8;
	goal_node.y = 9;
	int max_connect_distance = 3;

	rrt->Run_RRT(start_node, goal_node, max_connect_distance);

}

/**
 * @brief
 *
 *
 * **/
TEST_F(RRTTest, Run_RRTStar) {
	
	Node start_node;
	Node goal_node;
	start_node.x = 0;
	start_node.y = 0;
	start_node.DistanceFromStart = 0;
	goal_node.x = 8;
	goal_node.y = 9;
	int max_connect_distance = 3;
	int search_radius = 5;

	rrt->Run_RRTStar(start_node, goal_node, max_connect_distance, search_radius);
}

