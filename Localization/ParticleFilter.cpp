#include "ParticleFilter.hpp"

float ParticleFilter::Get_Range(VectorXf p1, VectorXf p2) {

	return sqrt(((p2[0] - p1[0])*(p2[0] - p1[0])) + ((p2[1] - p1[1])*(p2[1] - p1[1])));
}

float ParticleFilter::Get_Bearing(VectorXf p1, VectorXf p2) {

	float angle = atan2((p2[1] - p1[1]), (p2[0] - p1[0]));

	return (angle < 0) ? (angle += (2 * M_PI)): angle;
}


float ParticleFilter::Get_RandomBetween(float lowVal, float highVal) {   
    srand(time(NULL));
    float diff = highVal - lowVal;
    return (((float) rand() / RAND_MAX) * diff) + lowVal;
}

float ParticleFilter::ProbabilityDensityFunction(float x, Distribution distribution) {
	
	return (1 / (distribution.std_dev * std::sqrt(2 * M_PI)) * 
		std::exp(-0.5 * std::pow( (x - distribution.mean / distribution.std_dev), 2) ));
}


PointCloud ParticleFilter::Get_FeaturePoints(Particle particle) {

	PointCloud points_in_range;
	// Pseudo Scan: Check a given distance and width around the particle for feature point coordinates
	for (int x = 0; x < MapWidth; x++) {

		for (int y = 0; y < MapHeight; y++) {

			int cell = Map(x, y);
			if (cell > 0.0) {
				
				VectorXf map_feature(2);
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


void ParticleFilter::Generate_Weight(PointCloud robot_pointcloud, Particle &particle) {

	std::vector<Particle> ParticleSetUpdate;
	PointCloud simulated_pointcloud = Get_FeaturePoints(particle); // All occupied cells in range of particle

	// std::cout << "Size of Point Cloud in Range: " << simulated_pointcloud.points.size() << std::endl;
	// std::cout << "Particle: (" << particle.pose[0] << ", " << particle.pose[1] << ") Angle: " << particle.pose[2] << std::endl;

	float range_distribution_peak = 1 / (range_sigma * std::sqrt(2 * M_PI));
	float bearing_distribution_peak = 1 / (bearing_sigma * std::sqrt(2 * M_PI));

	// Generate new Weight
	float particle_weight = 0.0;
	// 1. Iterate through beams that Particle would see
	for (int p = 0; p < simulated_pointcloud.points.size(); p++) { 
		
		float beam_weight = 0.0;

		// 2. Iterate through Robot Beams in Scan
		for (int r = 0; r < robot_pointcloud.points.size(); r++) { 
			
			// Range Weight
			float robot_point_range = Get_Range(particle.pose, robot_pointcloud.points[r] * 100); // m --> cm
			float simulated_point_range = Get_Range(particle.pose, simulated_pointcloud.points[p]);
			Distribution normal_distro1 = Distribution(robot_point_range, range_sigma);
			float range_weight = ProbabilityDensityFunction(simulated_point_range, normal_distro1);

			//std::cout << "Range Weight: " << (float) range_weight << std::endl;
			
			// Normalize to 1
			range_weight /= range_distribution_peak;
			range_weight *= range_coef;

			// Bearing Weight
			float robot_point_bearing = Get_Bearing(particle.pose, robot_pointcloud.points[r] * 100); // m --> cm
			float simulated_point_bearing = Get_Bearing(particle.pose, simulated_pointcloud.points[p]);
			// Distribution normal_distro2 = Distribution(robot_point_bearing, bearing_sigma);
			// std::cout << "Robot Point Bearing: " << (float) robot_point_bearing << std::endl;
			// std::cout << "Simulated Point Bearing: " << (float) simulated_point_bearing << std::endl;
			// float bearing_weight = ProbabilityDensityFunction(simulated_point_bearing, normal_distro2);
			Distribution normal_distro2 = Distribution(0, bearing_sigma);
			float min_angle = abs(robot_point_bearing - simulated_point_bearing);
			if (min_angle > M_PI) { min_angle = abs(min_angle - (2 * M_PI)); } // Normalized	
			float bearing_weight = ProbabilityDensityFunction(min_angle, normal_distro2);


			// std::cout << "Bearing Weight: " << (float) bearing_weight << std::endl;
			
			// Normalize to 1
			bearing_weight /= bearing_distribution_peak;
			bearing_weight *= bearing_coef;

			// Beam Weight
			float weight = range_weight * bearing_weight;

			// std::cout << "Weight: " << (float) weight << std::endl;

			if (weight > beam_weight) { beam_weight = weight; }

		}

		// Final Particle Weight
		particle_weight += beam_weight;
		// Normalize particle weight based on num of beams in scan
		if (robot_pointcloud.points.size() > 0) { particle_weight /= robot_pointcloud.points.size(); }
		// Square particle weight so the best particles are more likely ot be resampled.
		particle_weight *= particle_weight;

		// std::cout << "Particle Weight: " << (float) particle_weight << std::endl;
	}

	particle.weight = particle_weight;
	// particle.weight = Get_RandomBetween(0, 1); // Temporary Test
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
	float rand = Get_RandomBetween(0.0, (1.f / (float) MaxParticles)); 
	int index = 0;
	float CumulativeDistro = ParticleSet[index].weight;
	std::cout << "Starting Cumulative: " << CumulativeDistro << std::endl;

	for (int m = 0; m < MaxParticles; m++) {
		
		float U = rand + ((float)m / (float)MaxParticles); // Increment U by 1/N
		std::cout << "rand: " << (float) rand << std::endl;
		std::cout << "U: " << (float) U << std::endl;
		
		while (U > CumulativeDistro && index < MaxParticles - 1) {
			
			index++;
			CumulativeDistro += ParticleSet[index].weight;
			
			std::cout << "New Cumulative: " << (float) CumulativeDistro << std::endl;
		}

		// std::cout << "Num of Particles: " << ParticleSet.size() << std::endl;
		// std::cout << "Index: " << index << std::endl;

		// Keep picking the particle while U is still within its weight range
		ResampledSet.push_back(ParticleSet[index]);
	}

	ParticleSet = ResampledSet;
}

ParticleFilter::ParticleFilter() {

	// Default Constructor
}


ParticleFilter::ParticleFilter(int max_particles, int pose_dimensions, float time_interval) : MaxParticles(max_particles), 
	PoseDimensions(pose_dimensions), TimeInterval(time_interval) {

	pose_sigma = 0.5;
	range_sigma = 0.5;
	bearing_sigma = 0.5;

	range_coef = 2;
	bearing_coef = 2;

	unsigned seed;

	// Uniformly Distribute Particles
	std::uniform_real_distribution<float> u_distro_x(0.0, MapWidth + 1);
	std::uniform_real_distribution<float> u_distro_y(0.0, MapHeight + 1);
	std::uniform_real_distribution<float> u_distro_th(0.0, 2 * M_PI);
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
 *  - 
 *
 *  - 
 *  
 *  - 
 *  */
