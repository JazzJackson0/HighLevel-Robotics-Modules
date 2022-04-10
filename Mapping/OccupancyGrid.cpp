#include "OccupancyGrid.hpp"


float OccupancyGridMap::InverseSensorModel(pair<int, int> cell, VectorXf pose, 
	vector<VectorXf> scan) {
	
	int x = cell.first;
	int y = cell.second;
	float range = sqrt(pow((x - pose[0]), 2) + pow((y - pose[1]), 2));
	float bearing = atan2( (y - pose[1]), (x - pose[0]) ) - pose[2];
	int k = Get_MinIndex(scan, bearing);
	
	float beam_range;
	if (MaxRange < scan[k][0] + Alpha / 2) { beam_range = MaxRange; }
	else { beam_range = scan[k][0] + Alpha / 2; }

	if (range > beam_range || abs(bearing - scan[k][1]) > Beta / 2) {
		return LogOdds(0.5);
	}

	else if (scan[k][0] < MaxRange && abs(range - MaxRange) < Alpha / 2) {
		return LogOdds(0.7);
	}

	else if (range <= scan[k][0]) {
		return LogOdds(0.3);
	}

}



float OccupancyGridMap::LogOdds(float x) {

	return (float) log( x / (1 - x) ); 
}



int OccupancyGridMap::Get_MinIndex(vector<VectorXf> scan, float bearing) {
	
	// Should't I add range comparison as well? Don't know, look over the
	// Inverse Sensor Model to see
	
	float min_dist = std::numeric_limits<float>::max();
	int index = -1;
	for (int i = 0; i < scan.size(); i++) {

		float dist = abs(scan[i][1] - bearing);

		if (dist < min_dist) { 
			
			min_dist = dist;
			index = i;
		}
	}

	return index;
}



OccupancyGridMap::OccupancyGridMap(int m, int n, float alpha, float beta, float max_range) {
	
	M = m;
	N = n;
	Alpha = alpha;
	Beta = beta;
	MaxRange = max_range;
	// Initialize the 3 Grid Matrices
}



void OccupancyGridMap::UpdateGridMap(VectorXf pose, vector<VectorXf> scan) {
	
	for (int i = 0; i < M; i++) {
		
		for (int j = 0; j < N; j++) {

			pair<int, int> cell = std::make_pair(i, j);
			GridMap[i][j] = InverseSensorModel(cell, pose, scan); // Is this how you access elements in a Matrix?

			GridMap = GridMap + PreviousGridMap - InitialGridMap;
			PreviousGridMap = GridMap;
		}
	}
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








