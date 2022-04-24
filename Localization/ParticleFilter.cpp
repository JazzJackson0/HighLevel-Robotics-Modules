#include "ParticleFilter.hpp"

vector<Particle> ParticleFilter::RunParticleFilter(vector<Particle> particle_set, 
	vector<vector<float>> scan, OdometryReadng odom) {
	
	
	
	vector<Particle> ParticleSetUpdate;
	float weightSum = 0.0;

	for (int m = 0; m < MaxParticles; m++) {
		
		float distribution_peak = 1 / (pose_std_dev * std::sqrt(2 * M_PI));

		// Calculate new Pose Mean using Control Command
			// Same motion models as other slam models
		vector<float> UpdatedPose(PoseDimensions);
		float trans = odom.RobotTranslation;
		float rot = odom.RobotRotation;
		UpdatedPose[0] = (particle_set[m].Pose[0] + (-1*(trans / rot)) * sin(particle_set[m].Pose[2]) 
			+ (-1*(trans / rot)) * sin(particle_set[m].Pose[2] + rot * TimeInterval) );
		UpdatedPose[1] = (particle_set[m].Pose[0] + (-1*(trans / rot)) * cos(particle_set[m].Pose[2]) 
			+ (-1*(trans / rot)) * cos(particle_set[m].Pose[2] + rot * TimeInterval) );
		UpdatedPose[2] = (particle_set[m].Pose[0] + rot * TimeInterval);

		// Generate Updated Sample Pose Estimate
		std::normal_distribution<float> x_distribution(UpdatedPose[0], pose_std_dev);
		std::normal_distribution<float> y_distribution(UpdatedPose[1], pose_std_dev);
		std::normal_distribution<float> th_distribution(UpdatedPose[2], pose_std_dev);
		
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine random_engine (seed);
		float new_x = x_distribution(random_engine);

		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine random_engine (seed);
		float new_y = y_distribution(random_engine);

		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine random_engine (seed);
		float new_angle = th_distribution(random_engine);

		// Filter out Landmarks not withing sensor range
		//... (Finish implementing)...
		vector<int> landmarks_in_range; // Index of Landmarks in range

		// Generate new Weight
		float particle_weight = 0.0;
		for (int p = 0; p < ; p++) { // Iterate through beams????????????
			
			float beam_weight = 0.0;

			for (int r = 0; r < scan.size(); r++) { // Iterate through Robot Beams in Scan

				// Convert Local Coordinates to Global Coordinates for scanned beam. DON'T THINK THIS IS NEEDED
				int sensor_x = (int) (scan[r][0] * cos(scan[r][1]));
				int sensor_y = (int) (scan[r][0] * sin(scan[r][1]));

				int x_global = ParticleSetUpdate[m].Pose[0] + 
					(cos(ParticleSetUpdate[m].Pose[2]) * sensor_x) - 
						(sin(ParticleSetUpdate[m].Pose[2]) * sensor_y);

				int y_global = ParticleSetUpdate[m].Pose[1] + 
					(sin(ParticleSetUpdate[m].Pose[2]) * sensor_x) - 
						(cos(ParticleSetUpdate[m].Pose[2]) * sensor_y);

				float beam_range = scan[r][0];
				float beam_bearing = scan[r][1];
				
				float range_weight = ProbabilityDensityFunction(beam_range, 
					particle_estimation, weight_std_dev);
				// Normalize to 1
				range_weight /= distribution_peak;
				range_weight *= range_coef;

				// Calculate min angle
				float min_angle = abs(beam_bearing - something); // The second should be particle angle
				if (min_angle > M_PI) { min_angle = abs(min_angle - (2 * M_PI)); }


				float bearing_weight = ProbabilityDensityFunction(min_angle, 
					particle_estimation, weight_std_dev);
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
	srand(time(NULL));
	float rand = (float) std::rand() / RAND_MAX ; // Obtain random number between 0 and 1
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



float ParticleFilter::ProbabilityDensityFunction(float robot_data, 
	float particle_data, float std_dev) {
	
	return (1 / (std_dev * std::sqrt(2 * M_PI)) * 
		std::exp(-0.5 * std::pow( (robot_data - particle_data / std_dev), 2) ));
}



ParticleFilter::ParticleFilter(int max_particles, int pose_dimensions, float time_interval) {
	
	MaxParticles = max_particles;
	PoseDimensions = pose_dimensions;
	TimeInterval = time_interval;

	// Uniformly Distribute Particles
	std::uniform_real_distribution<float> u_distro_x(0.0, MapWidth + 1);
	std::uniform_real_distribution<float> u_distro_y(0.0, MapHeight + 1);
	std::uniform_real_distribution<float> u_distro_th(0.0, 361);
	for (int i = 0; i < MaxParticles; i++) {

		Particle particle;

		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine random_engine (seed);
		particle.Pose.push_back(u_distro_x(random_engine)); // x

		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine random_engine (seed);
		particle.Pose.push_back(u_distro_y(random_engine)); // y

		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine random_engine (seed);
		particle.Pose.push_back(u_distro_th(random_engine)); // theta
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
