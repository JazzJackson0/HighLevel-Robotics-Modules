#pragma once
#include <iostream>
#include <random>
#include <chrono>
#include <time.h>
#include <vector>
#include <cmath>
using std::vector;
using std::pair;
using std::make_pair;

typedef struct particle Particle;
typedef struct odometry_reading OdometryReadng;

class ParticleFilter {

    private:
		vector<Particle> ParticleSet;
		int MaxParticles;
		int PoseDimensions;
		float TimeInterval;
		float pose_std_dev = 1.0;
		float weight_std_dev = 0.5;
		float range_coef; // How much range weight impacts total particle weight
		float bearing_coef; // How much bearing weight impacts total particle weight
		int MapWidth;
		int MapHeight;
        
		
		/**
         * @brief Runs the particle generation and importance weighting.
		 *
		 * @param particle_set The set of Particles
		 * @param motion_command The Odometry command (v, w)
		 * @param scan Range Scan
		 * @param odom The Odometry command (v, w)
         * 
         * @return ** vector<Particle> - Updated Particle Set
         */
        vector<Particle> RunParticleFilter(vector<Particle> particle_set, 
			vector<vector<float>> scan, OdometryReadng odom);


        /**
         * @brief Uses the Low Variance Resampling Algorithm to resample the particles.
		 *
		 * @param particle_set The set of Particles
         * 
         * @return ** vector<Particle> Resampled Particle Set 
         */
        vector<Particle> Resample(vector<Particle> particle_set);


		/**
		 * @brief Returns the probability of the mean (created with range_scan & particle_scan) 
		 * 			occurring. A higher value (used as a weight) indicates a higher similarity 
		 * 			between robot & particle data. A lower value indicates a low similarity.
		 *
		 * @param robot_data Range Scan 
		 * @param particle_data Particle Data
		 * @param std_dev Standard Deviation
		 *
		 * @return ** float - Probability
		 */
		float ProbabilityDensityFunction(float robot_data, float particle_data, float std_dev);


    public:

        /**
         * @brief Initializes the particle filter and creates a uniform distribution of particles.
		 *
		 * @param max_particles Max Number of Particles 
		 * @param pose_dimensions Robot/Particle pose dimensions
		 * @param time_interval Time interval ..........................
         */
        ParticleFilter(int max_particles, int pose_dimensions, float time_interval);


		/**
         * @brief Run the Particle Filter (Monte Carlo Localization) Algorithm.
         * 
         * @return ** void 
         */
		void RunMonteCarlo();
};



struct particle {
	vector<float> Pose;
	float weight;
};

struct odometry_reading {
	float RobotTranslation; 
	float RobotRotation;
};

