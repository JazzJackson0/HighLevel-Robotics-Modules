#include "ParticleFilter.hpp"


vector<Scan> ParticleFilter::GetFeaturePoints(Particle particle) {

	vector<Scan> points_in_range;
	// Pseudo Scan: Check a given distance and width around the particle for feature point coordinates
	for (int x = 0; x < MapWidth; x++) {

		for (int y = 0; y < MapHeight; y++) {

			int cell = Map[x][y];
			if (cell > 0.0) {

				float range = sqrt(((x - particle.Pose[0])*(x - particle.Pose[0])) + ((y - particle.Pose[1])*(y - particle.Pose[1])));
				float bearing = atan2(y, x) - particle.Pose[2];

				if (range > MaxBeamDist)
					continue;

				if (bearing > particle.Pose[2] + (AngularBeamWidth / 2) || bearing < particle.Pose[2] - (AngularBeamWidth / 2))
					continue;

				Scan scan;
				scan.range = range;
				scan.bearing = bearing;
				points_in_range.push_back(scan);
			}
		}
	}

	return points_in_range;
}


float ParticleFilter::ProbabilityDensityFunction(float robot_data, 
	float particle_data, float std_dev) {
	
	return (1 / (std_dev * std::sqrt(2 * M_PI)) * 
		std::exp(-0.5 * std::pow( (robot_data - particle_data / std_dev), 2) ));
}



void ParticleFilter::RunParticleFilter(vector<Scan> scan, OdometryReadng odom) {
	
	vector<Particle> ParticleSetUpdate;
	float weightSum = 0.0;
	unsigned seed;
	std::default_random_engine random_engine(seed);

	for (int m = 0; m < MaxParticles; m++) {
		
		float range_distribution_peak = 1 / (range_sigma * std::sqrt(2 * M_PI));
		float bearing_distribution_peak = 1 / (bearing_sigma * std::sqrt(2 * M_PI));

		// Calculate new Pose Mean using Control Command
			// Same motion models as other slam models
		vector<float> UpdatedPose(PoseDimensions);
		float trans = odom.RobotTranslation;
		float rot = odom.RobotRotation;
		UpdatedPose[0] = (ParticleSet[m].Pose[0] + (-1*(trans / rot)) * sin(ParticleSet[m].Pose[2]) 
			+ (-1*(trans / rot)) * sin(ParticleSet[m].Pose[2] + rot * TimeInterval) );
		UpdatedPose[1] = (ParticleSet[m].Pose[0] + (-1*(trans / rot)) * cos(ParticleSet[m].Pose[2]) 
			+ (-1*(trans / rot)) * cos(ParticleSet[m].Pose[2] + rot * TimeInterval) );
		UpdatedPose[2] = (ParticleSet[m].Pose[0] + rot * TimeInterval);

		// Generate Updated Sample Pose Estimate
		std::normal_distribution<float> x_distribution(UpdatedPose[0], pose_sigma);
		std::normal_distribution<float> y_distribution(UpdatedPose[1], pose_sigma);
		std::normal_distribution<float> th_distribution(UpdatedPose[2], pose_sigma);
		
		seed = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine random_engine(seed);
		float new_x = x_distribution(random_engine);

		seed = std::chrono::system_clock::now().time_since_epoch().count();
  		random_engine.seed(seed);
		float new_y = y_distribution(random_engine);

		seed = std::chrono::system_clock::now().time_since_epoch().count();
  		random_engine.seed(seed);
		float new_angle = th_distribution(random_engine);


		vector<Scan> cells_in_range = GetFeaturePoints(ParticleSet[m]); // All occupied cells in range

		// Generate new Weight
		float particle_weight = 0.0;
		for (int p = 0; p < cells_in_range.size(); p++) { // Iterate through beams that Particle would see
			
			float beam_weight = 0.0;

			for (int r = 0; r < scan.size(); r++) { // Iterate through Robot Beams in Scan
				
				// Get Global Coordinates from scan range & bearing
				int sensor_x = (int) ParticleSetUpdate[m].Pose[0] + (scan[r].range * cos(scan[r].bearing));
				int sensor_y = (int) ParticleSetUpdate[m].Pose[1] + (scan[r].range * sin(scan[r].bearing));
				
				/* Convert Local Coordinates to Global Coordinates for scanned beam.
				int sensor_x_global = ParticleSetUpdate[m].Pose[0] + 
					(cos(ParticleSetUpdate[m].Pose[2]) * sensor_x) - 
						(sin(ParticleSetUpdate[m].Pose[2]) * sensor_y);

				int sensor_y_global = ParticleSetUpdate[m].Pose[1] + 
					(sin(ParticleSetUpdate[m].Pose[2]) * sensor_x) - 
						(cos(ParticleSetUpdate[m].Pose[2]) * sensor_y);
				*/
				
				float range_weight = ProbabilityDensityFunction(scan[r].range, 
					cells_in_range[p].range, range_sigma);
				
				// Normalize to 1
				range_weight /= range_distribution_peak;
				range_weight *= range_coef;

				// Calculate min angle
				float min_angle = abs(scan[r].bearing - cells_in_range[p].bearing);
				if (min_angle > M_PI) { min_angle = abs(min_angle - (2 * M_PI)); }

				float bearing_weight = ProbabilityDensityFunction(0, 
					min_angle, bearing_sigma);
				
				// Normalize to 1
				bearing_weight /= bearing_distribution_peak;
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

	ParticleSet = ParticleSetUpdate;
}



void ParticleFilter::Resample() {

	vector<Particle> ResampledSet;
	srand(time(NULL));
	float rand = (float) std::rand() / RAND_MAX ; // Obtain random number between 0 and 1
	int index = 0;
	float CumulativeDistro = ParticleSet[index].weight;

	for (int m = 0; m < MaxParticles; m++) {
		
		float U = rand + (m / MaxParticles);
		
		while (U > CumulativeDistro) {
			
			index++;
			CumulativeDistro += ParticleSet[index].weight;
		}

		ResampledSet.push_back(ParticleSet[index]);
	}

	ParticleSet = ResampledSet;
}




ParticleFilter::ParticleFilter(int max_particles, int pose_dimensions, float time_interval) {
	
	MaxParticles = max_particles;
	PoseDimensions = pose_dimensions;
	TimeInterval = time_interval;
	unsigned seed;

	// Uniformly Distribute Particles
	std::uniform_real_distribution<float> u_distro_x(0.0, MapWidth + 1);
	std::uniform_real_distribution<float> u_distro_y(0.0, MapHeight + 1);
	std::uniform_real_distribution<float> u_distro_th(0.0, 361);
	for (int i = 0; i < MaxParticles; i++) {

		Particle particle;

		seed = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine random_engine(seed);
		particle.Pose.push_back(u_distro_x(random_engine)); // x

		seed = std::chrono::system_clock::now().time_since_epoch().count();
  		random_engine.seed(seed);
		particle.Pose.push_back(u_distro_y(random_engine)); // y

		seed = std::chrono::system_clock::now().time_since_epoch().count();
  		random_engine.seed(seed);
		particle.Pose.push_back(u_distro_th(random_engine)); // theta
		particle.weight = 1;
		ParticleSet.push_back(particle);
	}
} 


void ParticleFilter::AddMap(float ** map, float max_beam_dist, float angular_beam_width) {

	Map = map;
	MaxBeamDist = max_beam_dist;
	AngularBeamWidth = angular_beam_width;
}



void ParticleFilter::RunMonteCarlo(vector<Scan> scan, OdometryReadng odom) {

	RunParticleFilter(scan, odom);
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
