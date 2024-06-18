#include <iostream>
#include <gtest/gtest.h>
#include "../ScanMatching/ICP.hpp"
#include "../Algos/kd_tree.hpp"
 

class KDTreeTest : public ::testing::Test {

	protected:

		KDTreeTest() {
			
			float rand_val = 30.0;
			int pose_dimension = 2;
			bool switcher;

			// Setup 2 Point Clouds---------------

			// Old Point Cloud
			switcher = false;
			for (int i = 0; i < 30; i++) {
				
				VectorXf points(2);
				points << i, i + 1;  // (0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (), (), ()
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
			switcher = false;
			for (int i = 0; i < 6; i++) {
				
				VectorXf points(2);
				if (switcher) {
					points << i, i + 2;
					switcher = false;
				}

				else {
					points << i, i + 1;
					switcher = true;
				}
				
				p_cloud_new.Points.push_back(points);
				p_cloud_new.Weights.push_back(i * rand_val);
				
				// Just making some random number
				rand_val++;
				if ((int)rand_val % 5 == 00) {
					rand_val /= 5;
				}
			}
			
			kd_tree = new KDTree(pose_dimension);
		}
			
		~KDTreeTest() {
			delete kd_tree;
		}

		KDTree *kd_tree;
		PointCloud p_cloud_old;
		PointCloud p_cloud_new;
};


/**
 * brief
 *
 * **/
// TEST_F(KDTreeTest, build_tree) {
	
// 	struct Node *tree = kd_tree->build_tree(p_cloud_old.Points);
	
// }

/**
 * brief
 *
 * **/
// TEST_F(KDTreeTest, insert) {

// 	struct Node *tree = kd_tree->build_tree(p_cloud_old.Points);
// 	VectorXf point(2);
// 	point << 3, 57;
// 	kd_tree->insert(point, 0, tree, 0);
	
// }

/**
 * brief
 *
 * **/
TEST_F(KDTreeTest, remove) {

	struct Node *tree = kd_tree->build_tree(p_cloud_new.Points);
	VectorXf point_t(2);
	point_t << 2, 3;
	VectorXf point_f(2);
	point_f << 3, 57;

	struct Node *search_res = kd_tree->search(point_t, tree, 0);
	EXPECT_EQ(search_res->pos, 2);
	EXPECT_EQ(search_res->data[0], 2); 
	EXPECT_EQ(search_res->data[1], 3); 


	struct Node *new_tree = kd_tree->remove(point_t, tree, 0);
	struct Node *new_tree2 = kd_tree->remove(point_f, tree, 0);

	search_res = kd_tree->search(point_t, new_tree, 0);
	EXPECT_EQ(search_res, nullptr);
}


/**
 * brief
 *
 * **/
TEST_F(KDTreeTest, search) {
	
	struct Node *tree = kd_tree->build_tree(p_cloud_old.Points);
	struct Node *tree2 = kd_tree->build_tree(p_cloud_new.Points);
	VectorXf point(2);
	point << 0, 1;
	VectorXf point2(2);
	point2 << 3, 5;

	struct Node *res = kd_tree->search(point, tree, 0);
	struct Node *res2 = kd_tree->search(point2, tree2, 0);

	EXPECT_EQ(res->pos, 0);
	EXPECT_EQ(res->data[0], 0); 
	EXPECT_EQ(res->data[1], 1); 
	
	EXPECT_EQ(res2->pos, 3);
	EXPECT_EQ(res2->data[0], 3); 
	EXPECT_EQ(res2->data[1], 5); 
}

/**
 * brief
 *
 * **/
// TEST_F(KDTreeTest, find_min) {
	
// 	struct Node *tree = kd_tree->build_tree(p_cloud_old.Points);
// 	VectorXf point(2);
// 	point << 3, 57;
	
// 	EXPECT_EQ(kd_tree->find_min(tree)->data[0], 2); 
// 	EXPECT_EQ(kd_tree->find_min(tree)->data[1], 2); 
// }


/**
 * brief
 *
 * **/
TEST_F(KDTreeTest, get_nearest_neighbor) {

	struct Node *tree = kd_tree->build_tree(p_cloud_old.Points);
	struct Node *tree2 = kd_tree->build_tree(p_cloud_new.Points);
	VectorXf point(2);
	point << 5, 5;
	VectorXf point2(2);
	point2 << 2, 4;
	
	EXPECT_EQ(kd_tree->get_nearest_neighbor(point, tree, 0)->data[0], 3); 
	EXPECT_EQ(kd_tree->get_nearest_neighbor(point, tree, 0)->data[1], 4); 


	EXPECT_EQ(kd_tree->get_nearest_neighbor(point2, tree2, 0)->data[0], 2); 
	EXPECT_EQ(kd_tree->get_nearest_neighbor(point2, tree2, 0)->data[1], 3); 
	
}





