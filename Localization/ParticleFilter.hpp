#pragma once
#include <iostream>
#include <random>
#include <chrono>
#include <time.h>
#include <vector>
#include <cmath>
#include "utils.hpp"
#include </usr/include/eigen3/Eigen/Dense>
#include "/usr/include/eigen3/unsupported/Eigen/CXX11/Tensor"
using std::vector;
using std::pair;
using std::make_pair;
using namespace Eigen;



struct Particle {
	VectorXf pose;
	float weight;
};


class ParticleFilter {

    private:
		std::vector<Particle> ParticleSet;
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
		Eigen::Tensor<float, 2> Map;
		int MapWidth;
		int MapHeight;
        

		/**
		 * @brief 
		 * 
		 * @param p1 
		 * @param p2 
		 * @return float 
		 */
		float Get_Range(VectorXf p1, VectorXf p2);


		/**
		 * @brief 
		 * 
		 * @param p1 
		 * @param p2 
		 * @return * float 
		 */
		float Get_Bearing(VectorXf p1, VectorXf p2);


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
		 * @brief Look through each occupied cell in the map and determine if it falls within the range & bounds of the scanner.
		 * 			 If so, add that cell to an array.
		 * 
		 * @param particle The particle around which feature points will be checked for.
		 * 
		 * @return ** Scan - Returns a vector of all the feature points picked up within range of the give particle
		 */
		PointCloud Get_FeaturePoints(Particle particle);


		/**
		 * @brief 
		 * 
		 * @param particle_idx 
		 * @param odom 
		 * @return Particle 
		 */
		Particle Move_Particle(int particle_idx, ControlCommand odom);


		/**
		 * @brief Generates a weight value for a single particle by comparing the similarity between the point cloud from 
		 * 		the scanner and the estimated point cloud for the particle at 'particle_idx' 
		 * 
		 * @param current_pointcloud The point cloud obtained from the scanner.
		 * @param particle The particle to be weighted. 
		 */
		void Generate_Weight(PointCloud current_pointcloud, Particle &particle);

		
		/**
         * @brief Runs the particle generation and importance weighting.
		 *
		 * @param scan Robot Scan
		 * @param odom The Odometry output (v, w)
         * 
         * @return ** void
         */
        void RunParticleFilter(PointCloud scan, ControlCommand odom);


        /**
         * @brief Uses the Low Variance Resampling Algorithm to resample the particles.
         * 
         * @return ** void 
         */
        void Resample();



    public:

		/**
		 * @brief Default Constructor
		 * 
		 */
		ParticleFilter();

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
		void AddMap(Eigen::Tensor<float, 2> map, float max_beam_dist, float angular_beam_width);


		/**
		 * @brief Run the Particle Filter (Monte Carlo Localization) Algorithm.
		 * 
		 * @param scan 
		 * @param odom 
		 */
		void Run(PointCloud scan, ControlCommand odom);
};





