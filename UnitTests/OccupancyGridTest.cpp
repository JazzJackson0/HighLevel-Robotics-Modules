#include <iostream>
#include <gtest/gtest.h>
#include "OccupancyGrid.hpp"


class OccupancyGridMapTest : public ::testing::Test {

	protected:

		OccupancyGridMapTest() {

			int row_num = 100;
			int col_num = 100;
			int alpha = 1;
			int beta = 2;
			int max_scan_range = 5;
			occupancy_grid = new OccupancyGridMap(row_num, col_num, alpha, beta, max_scan_range);
		}
			
		~OccupancyGridMapTest() {
			delete occupancy_grid;
		}

		OccupancyGridMap *occupancy_grid;
};

 
/**
 * @brief
 *
 * **/
TEST_F(OccupancyGridMapTest, UpdateGridMap1) {

	vector<VectorXf> scan(10);
	VectorXf pose(3);
	pose(0) = 1.5;
	pose(1) = 2.5;
	pose(2) = 3.5;

	// Create vector of 10 scan points.
	for (int i = 0; i < 10; i++) {
		VectorXf scan_i(3);
		scan_i << i, i + 1, i + 2;
		scan.push_back(scan_i);		
	}

	occupancy_grid->UpdateGridMap(pose, scan);
	
}

