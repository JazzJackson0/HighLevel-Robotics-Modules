#include "OccupancyGrid.hpp"


		float OccupancyGridMap::InverseSensorModel(pair<int, int> cell, VectorXf pose, vector<VectorXf> measurements) {
			
			int x = cell.first;
			int y = cell.second;
			float range = sqrt(pow((x - pose[0]), 2) + pow((y - pose[1]), 2));
			float bearing = atan2( (y - pose[1]), (x - pose[0]) ) - pose[2];
			int k = Get_MinIndex(measurements, bearing);
			
			float beam_range;
			if (MaxRange < measurements[k][0] + alpha / 2) { beam_range = MaxRange; }
			else { beam_range = measurements[k][0] + alpha / 2; }

			if (range > beam_range || abs(bearing - measurements[k][1]) > beta / 2) {
				return LogOdds(0.5);
			}

			else if (measurements[k][0] < MaxRange && abs(range - MaxRange) < alpha / 2) {
				return LogOdds(0.7);
			}

			else if (range <= measurements[k][0]) {
				return LogOdds(0.3);
			}

		}



		float OccupancyGridMap::LogOdds(float x) {

			return (float) log( x / (1 - x) ); 
		}



		int OccupancyGridMap::Get_MinIndex(vector<VectorXf> measurements, float bearing) {
			
			float min_dist = std::numeric_limits<float>::max();
			int index = -1;
			for (int i = 0; i < measurements.size(); i++) {

				float dist = abs(vector[i][1] - bearing);

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



		void OccupancyGridMap::UpdateGridMap(VectorXf pose, vector<VectorXf> measurements) {
			
			for (int i = 0; i < M; i++) {
				
				for (int j = 0; j < N; j++) {

					pair<int, int> cell = std::make_pair(i, j);
					GridMap[i][j] = InverseSensorModel(cell, pose, measurements); // Is this how you access elements in a Matrix?

					GridMap = GridMap + PreviousGridMap - InitialGridMap;
					PreviousGridMap = GridMap;
				}
			}
		}
};



/*
 * 			TO-DO
 * 			-----
 *  - Finish
 *
 *  - Test Code
 *  
 *  - 
 *  */








