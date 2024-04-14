#include <iostream>
#include <gtest/gtest.h>
#include "ParticleFilter.hpp"


class ParticleFilterTest : public ::testing::Test {

	protected:

		ParticleFilterTest() {

			int max_particles = 50;
			int pose_dimensions = 3; 
			int time_interval = 1;
			pfilter = new ParticleFilter(max_particles, pose_dimensions, time_interval);
		}
			
		~ParticleFilterTest() {
			delete pfilter;
		}

		ParticleFilter *pfilter;
};


/**
 * @brief
 *
 * **/
TEST_F(ParticleFilterTest, RunMonteCarlo1) {

	vector<Scan> scan;
	OdometryReadng odom;
	pfilter->RunMonteCarlo(scan, odom);
}
