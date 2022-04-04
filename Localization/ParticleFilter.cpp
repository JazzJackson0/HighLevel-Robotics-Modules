#include "ParticleFilter.hpp"
#include <cmath>
#include <random>
		

        vector<Particle> ParticleFilter::RunParticleFilter(vector<Particle> particle_set, vector<float> control_command, vector<vector> scan) {
            
			vector<Particle> ParticleSetUpdate;
			float weightSum = 0.0;

			for (int m = 0; m < MaxParticles; m++) {
				
				float distribution_peak = 1 / (pose_std_dev * std::sqrt(2 * M_PI));

				// Calculate new Pose Mean using Control Command
					// Same motion models as other slam models

				// Generate Updated Sample Pose Estimate
				distribution(x, pose_std_dev);
				float new_x = distribution(generator);
				distribution(y, pose_std_dev);
				float new_y = distribution(generator);
				distribution(angle, pose_std_dev);
				float new_angle = distribution(generator);

				// Generate new Weight
				float particle_weight = 0.0;
				for (int p = 0; p < sdfsd; p++) { // Iterate through ... ?????????????
					
					float beam_weight = 0.0;

					for (int r = 0; r < scan.size(); r++) { // Iterate through Robot Beams in Scan
					
						float range_weight = ProbabilityDensityFunction(scan[r][0], float particle_estimation, weight_std_dev);
						// Normalize to 1
						range_weight /= distribution_peak;
						range_weight *= range_coef;

						// Calculate min angle
						float min_angle = abs(scan[r][1] - sldkfjsdf);
						if (min_angle > M_PI) { min_angle = abs(min_angle - (2 * M_PI)); }


						float bearing_weight = ProbabilityDensityFunction(min_angle, float particle_estimation, weight_std_dev);
						// Normalize to 1
						bearing_weight /= distribution_peak;
						bearing_weight *= bearing_coef;

						float weight = range_weight * bearing_weight;
						if (weight > beam_weight) { beam_weight = weight; }

					particle_weight += beam_weight;

					// Normalize weights based on num of beams in scan
					if (scan.size() > 0) { particle_weight /= scan.size(); }

					// Square weight so the best particles are more likely ot be resampled.
					particle_weight *= particle_weight;

					}
				}

				// Add pose and weight to Updated Set
				ParticleSetUpdate[m].Pose.push_back(new_x);
				ParticleSetUpdate[m].Pose.push_back(new_y);
				ParticleSetUpdate[m].Pose.push_back(new_angle);
				ParticleSetUpdate[m].weight = particle_weight;

			}

			return ParticleSetUpdate;
        }

		

        vector<Particle> ParticleFilter::Resample(vector<Particle> particle_set) {

        	vector<Particle> ResampledSet;
			float rand = ; // Obtain random number
			int index = 0;
			float CumulativeDistro = particle_set[index].weight;

			for (int m = 0; m < MaxParticles; m++) {
				
				float U = rand + (m / MaxParticles);
				
				while (U > CumulativeDistro) {
					
					index++;
					CumulativeDistro += particle_set[index].weight;
				}

				ResampledSet.push_back(particle_set[index]);
			}

			return ResampledSet;
		}



		float ParticleFilter::ProbabilityDensityFunction(float robot_scan, float particle_scan, float std_dev) {
			
			return (1 / (std_dev * std::sqrt(2 * M_PI)) * std::exp(-0.5 * std::pow( (robot_scan - particle_scan/std_dev), 2) );
		}



		ParticleFilter::ParticleFilter(int max_particles) {
			
			MaxParticles = max_particles;

			// Uniformly Distribute Particles
			std::uniform_real_distribution<float> u_distro(0.0, pose_std_dev); // likely need to tweak parameters
			for (int i = 0; i < MaxParticles; i++) {

				Particle particle;
				particle.Pose.push_back(u_distro(generator)); // x
				particle.Pose.push_back(u_distro(generator)); // y
				particle.Pose.push_back(u_distro(generator)); // theta
				particle.weight = 1;
				ParticleSet.push_back(particle);
			}


        } 
		
	

		void ParticleFilter::RunMonteCarlo() {
			
			RunParticleFilter();
			Resample();
		}









/*
 * 			TO-DO
 * 			-----
 *  - Finish
 *
 *  - Test Code
 *  
 *  - 
 *  */
