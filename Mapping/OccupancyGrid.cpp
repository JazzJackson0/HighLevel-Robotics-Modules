#include "OccupancyGrid.hpp"

// Private-------------------------------------------------------------------------------------------------------------------------------------
inline float OccupancyGridMap::LogOdds(float x) {

	return (float) log( x / (1 - x) ); 
}


float OccupancyGridMap::InverseSensorModel(ogrid::Cell cell, std::vector<VectorXf> beams) {

	float cell_range = hypot((cell.x - Pose[0]), (cell.y - Pose[1]));
	float cell_bearing = atan2( (cell.y - Pose[1]), (cell.x - Pose[0]) ) - PoseAngle;
	
	int k = Get_MostSimilarBeam(beams, cell, cell_range, cell_bearing);

	if (k < 0) return LogOdds(0.0);

	//std::cout << "k: " << k << std::endl;
	
	// Determine Range of Beam k
	float beam_range;
	if (beams[k][0] + (Alpha / 2) > MaxRange) beam_range = MaxRange; 
	else beam_range = beams[k][0] + (Alpha / 2); 

	// IF Cell is farther than Beam range [OR] Cell is out of the scanner's field of view
	if (cell_range > beam_range || abs(cell_bearing - beams[k][1]) > (Beta / 2))
		return LogOdds(0.5); // Don't know whether occupied or not.

	// IF Cell is closer than Beam range
	else if (cell_range <= beams[k][0])
		return LogOdds(0.3); // Most likely NOT occupied

	// IF Cell range is about the same as beam range
	else if (beams[k][0] < MaxRange && abs(cell_range - MaxRange) < (Alpha / 2))
		return LogOdds(0.7); // Most likely occupied

	return LogOdds(0.0);
}


int OccupancyGridMap::Get_MostSimilarBeam(std::vector<VectorXf> beams, ogrid::Cell cell, float cell_range, float cell_bearing) {

	float min_heading = std::numeric_limits<float>::max();
	int index = -1;
	for (int i = 0; i < beams.size(); i++) {

		float heading = 0.f;
		heading += abs(beams[i][0] - cell_range); // Range Difference
		heading += abs(beams[i][1] - cell_bearing); // Angle Difference

		if (heading < min_heading) { 
			min_heading = heading;
			index = i;
		}
	}

	return index;
}

// Public-------------------------------------------------------------------------------------------------------------------------------------

OccupancyGridMap::OccupancyGridMap() { /** Default Constructor */ }

OccupancyGridMap::OccupancyGridMap(int m, int n, float alpha, float beta, float max_range) : 
	M(m), N(n), Alpha(alpha), Beta(beta), MaxRange(max_range) {
	
	Eigen::Tensor<float, 2> g(M, N);
	g.setZero();
	GridMap = g;
	map_builder = MapBuilder(M, N);
	PoseAngle = 0;
}



Eigen::Tensor<float, 2> OccupancyGridMap::UpdateGridMap(VectorXf pose, std::vector<VectorXf> scan) {

	// auto start = std::chrono::high_resolution_clock::now();

	Pose = map_builder.MapCoordinate_to_DataStructureIndex(pose.head<2>());
	PoseAngle = pose[2];
	
	#pragma omp parallel for collapse(2)
	for (int i = 0; i < M; i++) {
		
		for (int j = 0; j < N; j++) {

			ogrid::Cell cell;
			cell.x = i;
			cell.y = j;
			GridMap(i, j) += InverseSensorModel(cell, scan);
		}
	}

	// auto end = std::chrono::high_resolution_clock::now();
	// std::cout << "Map Made!!!!" << "\n";
	// std::cout << "Map Build Time: " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " sec" << std::endl;

	return GridMap;
}


Eigen::Tensor<float, 2> OccupancyGridMap::UpdateGridMapWithPointCloud(PointCloud cloud) {
	
	for (int i = 0; i < cloud.points.size(); i++) {

		VectorXf pt(2);
		pt << (cloud.points[i][0]), (cloud.points[i][1]);

		VectorXi index = map_builder.MapCoordinate_to_DataStructureIndex(pt);
		
		GridMap(index[0], index[1]) = 1.f;
	}

	return GridMap;
}



/*
 * 			TO-DO
 * 			-----
 *  - Inverse Sensor Model (Really, the Get_MostSimilarBeam() func) takes waaaay too long lol. I'll drastically overhaul the performance later
 *
 *  - 
 *  
 *  - 
 *  */








