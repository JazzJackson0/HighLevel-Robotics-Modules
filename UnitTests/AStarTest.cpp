#include <iostream>
#include <gtest/gtest.h>
#include "AStar.hpp"


class AStarTest : public ::testing::Test {

	protected:

		AStarTest() {

			int grid_width = 10;
			int grid_height = 8;
			int pre_grid[grid_height][grid_width] = {{1, 1, 1, 0, 1, 1, 0, 0, 1, 1},
													{0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
													{0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
													{0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
													{0, 0, 0, 0, 1, 1, 1, 0, 0, 1},
													{0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
													{0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
													{0, 0, 0, 0, 0, 1, 1, 1, 0, 0}};
			int* grid_rows[grid_height] = {pre_grid[0], pre_grid[1], pre_grid[2], pre_grid[3], pre_grid[4], pre_grid[5], pre_grid[6], pre_grid[7]};
			int** grid = grid_rows;
			
			astar = new A_Star(grid, grid_width, grid_height);
		}
			
		//~AStarTest() {}

		A_Star *astar;
};


/**
 *	@brief 
 *
 * **/
TEST_F(AStarTest, Search1) {
	
	CellCoordinate start = std::make_pair(0, 5);
	CellCoordinate goal = std::make_pair(9, 9);
	EXPECT_TRUE(astar->Search(start, goal));
}
