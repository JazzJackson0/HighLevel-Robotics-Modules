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
typedef struct scan Scan;

class ParticleFilter {

    private:
		vector<Particle> ParticleSet;
		float MaxBeamDist;
		float AngularBeamWidth;
		int MaxParticles;
		int PoseDimensions;
		float TimeInterval;
		float pose_sigma = 1.0; // Pose Standard Deviation
		float range_coef; // How much range weight impacts total particle weight
		float range_sigma = 1.0; // Standard Deviation for the Range
		float bearing_coef; // How much bearing weight impacts total particle weight
		float bearing_sigma = 0.5; // Standard Deviation for the Bearing
		
		// Map
		float **Map;
		int MapWidth;
		int MapHeight;
        

		/**
		 * @brief Look throuch each occupied cell in the map and determine if it falls within the range & bounds of the scanner.
		 * 			 If so, add that cell to an array.
		 * 
		 * @param particle The particle around which feature points will be checked for.
		 * 
		 * @return ** Scan - Returns a vector of all the feature points picked up within range of the give particle
		 */
		vector<Scan> GetFeaturePoints(Particle particle);


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

		
		/**
         * @brief Runs the particle generation and importance weighting.
		 *
		 * @param scan Robot Scan
		 * @param odom The Odometry command (v, w)
         * 
         * @return ** void
         */
        void RunParticleFilter(vector<Scan> scan, OdometryReadng odom);


        /**
         * @brief Uses the Low Variance Resampling Algorithm to resample the particles.
         * 
         * @return ** void 
         */
        void Resample();



    public:

        /**
         * @brief Initializes the particle filter and creates a uniform distribution of particles.
		 *
		 * @param max_particles Max Number of Particles 
		 * @param pose_dimensions Robot/Particle pose dimensions
		 * @param time_interval Time interval ..........................
		 * @param search_dist The distance from the particle to search for map features
		 * @param search_width The Width of the range around the particle to search for map features
         */
        ParticleFilter(int max_particles, int pose_dimensions, float time_interval);

		/**
		 * @brief Add the map that the Particle Filter will localize the robot in.
		 * 
		 * @param map Map used to localize the robot in.
		 * @param max_beam_dist The distance from the particle to search for map features
		 * @param angular_beam_width The Width of the range around the particle to search for map features
		 */
		void AddMap(float ** map, float max_beam_dist, float angular_beam_width);


		/**
		 * @brief Run the Particle Filter (Monte Carlo Localization) Algorithm.
		 * 
		 * @param scan 
		 * @param odom 
		 */
		void RunMonteCarlo(vector<Scan> scan, OdometryReadng odom);
};



struct particle {
	vector<float> Pose;
	float weight;
};

struct odometry_reading {
	float RobotTranslation; 
	float RobotRotation;
};

struct scan {
	float range;
	float bearing;
};

