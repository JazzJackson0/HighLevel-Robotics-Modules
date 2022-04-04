#pragma once
#include <iostream>
#include <random>
#include <vector>
using std::vector;

typedef struct particle Particle;

class ParticleFilter {

    private:
		std::default_random_engine generator;
		std::normal_distribution<float> distribution();
		vector<Particle> ParticleSet;
		int MaxParticles;
		float pose_std_dev = 1.0;
		float weight_std_dev = 0.5;
		float range_coef; // How much range weight impacts total particle weight
		float bearing_coef; // How much bearing weight impacts total particle weight
        
		
		/**
         * @brief
		 *
		 * @param particle_set
		 * @param control_command
		 * @oaram scan
         * 
         * @return ** vector<Particle> - Updated Particle Set
         */
        vector<Particle> RunParticleFilter(vector<Particle> particle_set, vector<float> control_command, vector<vector> scan);


        /**
         * @brief Low Variance Resampling
		 *
		 * @param particle_set
         * 
         * @return ** vector<Particle> Resampled Particle Set 
         */
        vector<Particle> Resample(vector<Particle> particle_set);


		/**
		 * @brief 
		 *
		 * @param robot_scan
		 * @param particle_scan
		 * @param std_dev
		 *
		 * @return float - Probability
		 *
		 * **/
		float ProbabilityDensityFunction(float robot_scan, float particle_scan, float std_dev);


    public:

        /**
         * @brief 
		 *
		 * @param max_particles
         * 
         */
        ParticleFilter(int max_particles);


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

