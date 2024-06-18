#include "ParticleFilter.hpp"

float ParticleFilter::Get_Range(VectorXf p1, VectorXf p2) {

	return sqrt(((p2[0] - p1[0])*(p2[0] - p1[0])) + ((p2[1] - p1[1])*(p2[1] - p1[1])));
}

float ParticleFilter::Get_Bearing(VectorXf p1, VectorXf p2) {

	return atan2((p2[1] - p1[1]), (p2[0] - p1[0]));
}

float ParticleFilter::ProbabilityDensityFunction(float robot_data, 
	float particle_data, float std_dev) {
	
	return (1 / (std_dev * std::sqrt(2 * M_PI)) * 
		std::exp(-0.5 * std::pow( (robot_data - particle_data / std_dev), 2) ));
}


PointCloud ParticleFilter::Get_FeaturePoints(Particle particle) {

	PointCloud points_in_range;
	// Pseudo Scan: Check a given distance and width around the particle for feature point coordinates
	for (int x = 0; x < MapWidth; x++) {

		for (int y = 0; y < MapHeight; y++) {

			int cell = Map(x, y);
			if (cell > 0.0) {
				
				VectorXf map_feature(PoseDimensions);
				map_feature << (float) x, (float) y;
				float range = Get_Range(map_feature, particle.pose);
				float bearing = Get_Bearing(map_feature, particle.pose);

				if (range > MaxBeamDist)
					continue;

				if (bearing > particle.pose[2] + (AngularBeamWidth / 2) || bearing < particle.pose[2] - (AngularBeamWidth / 2))
					continue;

				points_in_range.points.push_back(map_feature);
			}
		}
	}

	return points_in_range;
}



Particle ParticleFilter::Move_Particle(int particle_idx, ControlCommand odom) {

	Particle updated_particle;
	updated_particle.pose = VectorXf::Zero(PoseDimensions);
	float trans = odom.trans_vel;
	float rot = odom.rot_vel;
	
	updated_particle.pose[0] = (ParticleSet[particle_idx].pose[0] + (-1*(trans / rot)) * sin(ParticleSet[particle_idx].pose[2]) 
		+ (-1*(trans / rot)) * sin(ParticleSet[particle_idx].pose[2] + rot * TimeInterval) );
	
	updated_particle.pose[1] = (ParticleSet[particle_idx].pose[0] + (-1*(trans / rot)) * cos(ParticleSet[particle_idx].pose[2]) 
		+ (-1*(trans / rot)) * cos(ParticleSet[particle_idx].pose[2] + rot * TimeInterval) );
	
	updated_particle.pose[2] = (ParticleSet[particle_idx].pose[0] + rot * TimeInterval);

	return updated_particle;
}


void ParticleFilter::Generate_Weight(PointCloud current_pointcloud, 
	Particle &particle) {

	std::vector<Particle> ParticleSetUpdate;
	PointCloud feature_cloud_in_range = Get_FeaturePoints(particle); // All occupied cells in range of particle

	float range_distribution_peak = 1 / (range_sigma * std::sqrt(2 * M_PI));
	float bearing_distribution_peak = 1 / (bearing_sigma * std::sqrt(2 * M_PI));

	// Generate new Weight
	float particle_weight = 0.0;
	// 1. Iterate through beams that Particle would see
	for (int p = 0; p < feature_cloud_in_range.points.size(); p++) { 
		
		float beam_weight = 0.0;

		// 2. Iterate through Robot Beams in Scan
		for (int r = 0; r < current_pointcloud.points.size(); r++) { 
			
			// Range Weight
			float current_point_range = Get_Range(particle.pose, current_pointcloud.points[r]);
			float feature_point_range = Get_Range(particle.pose, feature_cloud_in_range.points[p]);
			float range_weight = ProbabilityDensityFunction(current_point_range, feature_point_range, range_sigma);
			
			// Normalize to 1
			range_weight /= range_distribution_peak;
			range_weight *= range_coef;

			// Bearing Weight
			float current_point_bearing = Get_Bearing(particle.pose, current_pointcloud.points[r]);
			float feature_point_bearing = Get_Bearing(particle.pose, feature_cloud_in_range.points[p]);
			float min_angle = abs(current_point_bearing - feature_point_bearing);
			if (min_angle > M_PI) { min_angle = abs(min_angle - (2 * M_PI)); }
			float bearing_weight = ProbabilityDensityFunction(0, min_angle, bearing_sigma);
			
			// Normalize to 1
			bearing_weight /= bearing_distribution_peak;
			bearing_weight *= bearing_coef;

			// Beam Weight
			float weight = range_weight * bearing_weight;
			if (weight > beam_weight) { beam_weight = weight; }

		}

		// Final Particle Weight
		particle_weight += beam_weight;
		// Normalize particle weight based on num of beams in scan
		if (current_pointcloud.points.size() > 0) { particle_weight /= current_pointcloud.points.size(); }
		// Square particle weight so the best particles are more likely ot be resampled.
		particle_weight *= particle_weight;
	}

	particle.weight = particle_weight;
}


void ParticleFilter::RunParticleFilter(PointCloud scan, ControlCommand odom) {
	
	float weightSum = 0.0;
	unsigned seed;
	std::default_random_engine random_engine(seed);
	
	for (int m = 0; m < MaxParticles; m++) {
			
		// 1. Calculate new location for moved Particle (New Pose Mean)
		Particle moved_particle = Move_Particle(m, odom);	
		Particle moved_particle_variation;
		moved_particle_variation.pose = VectorXf::Zero(PoseDimensions);

		// Generate Updated Sample Pose Estimate
		std::normal_distribution<float> normal_x_distribution(moved_particle.pose[0], pose_sigma);
		std::normal_distribution<float> normal_y_distribution(moved_particle.pose[1], pose_sigma);
		std::normal_distribution<float> normal_th_distribution(moved_particle.pose[2], pose_sigma);
		
		// 2. Apply slight random variation to the moved particle pose
		seed = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine random_engine(seed);
		moved_particle_variation.pose[0] = normal_x_distribution(random_engine); // Set Random x

		seed = std::chrono::system_clock::now().time_since_epoch().count();
  		random_engine.seed(seed);
		moved_particle_variation.pose[1] = normal_y_distribution(random_engine); // Set Random y 

		seed = std::chrono::system_clock::now().time_since_epoch().count();
  		random_engine.seed(seed);
		moved_particle_variation.pose[2] = normal_th_distribution(random_engine); // Set Random theta

		Generate_Weight(scan, moved_particle_variation);

		// Update Particle Set with new random particle
		ParticleSet[m] = moved_particle_variation;
	}
}



void ParticleFilter::Resample() {

	std::vector<Particle> ResampledSet;
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

ParticleFilter::ParticleFilter() {

	// Default Constructor
}


ParticleFilter::ParticleFilter(int max_particles, int pose_dimensions, float time_interval) : MaxParticles(max_particles), 
	PoseDimensions(pose_dimensions), TimeInterval(time_interval) {

	unsigned seed;

	// Uniformly Distribute Particles
	std::uniform_real_distribution<float> u_distro_x(0.0, MapWidth + 1);
	std::uniform_real_distribution<float> u_distro_y(0.0, MapHeight + 1);
	std::uniform_real_distribution<float> u_distro_th(0.0, 361);
	for (int i = 0; i < MaxParticles; i++) {

		Particle particle;
		particle.pose = VectorXf::Zero(PoseDimensions);

		seed = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine random_engine(seed);
		particle.pose [0] = u_distro_x(random_engine); // x

		seed = std::chrono::system_clock::now().time_since_epoch().count();
  		random_engine.seed(seed);
		particle.pose[0] = u_distro_y(random_engine); // y

		seed = std::chrono::system_clock::now().time_since_epoch().count();
  		random_engine.seed(seed);
		particle.pose[0] = u_distro_th(random_engine); // theta
		
		particle.weight = 1 / MaxParticles; // weight
		ParticleSet.push_back(particle);
	}
} 


void ParticleFilter::AddMap(Eigen::Tensor<float, 2> map, float max_beam_dist, float angular_beam_width) {

	Map = map;
	const auto& d = Map.dimensions();
	MapWidth = d[1];
	MapHeight = d[0];
	MaxBeamDist = max_beam_dist;
	AngularBeamWidth = angular_beam_width;
}



void ParticleFilter::Run(PointCloud scan, ControlCommand odom) {

	RunParticleFilter(scan, odom);
	Resample();
}



/*
 * 			TO-DO
 * 			-----
 *  - Test Code
 *
 *  - 
 *  
 *  - 
 *  */
