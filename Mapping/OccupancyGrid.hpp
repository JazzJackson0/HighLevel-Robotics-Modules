#include <iostream>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <omp.h>
#include <chrono>
#include "unsupported/Eigen/CXX11/Tensor"
#include "utils.hpp"
#include "MapBuilder.hpp"

using namespace::Eigen;
using std::vector;
using std::pair;
using std::make_pair;

namespace ogrid{
struct Cell {
    int x, y;
}; 
};


class OccupancyGridMap {

	private:
		Eigen::Tensor<float, 2> GridMap;
		VectorXi Pose; // Current pose of the robot
		float PoseAngle;
		int M;
		int N;
		float Alpha; // Width of Cell
		float Beta; // Angular width of Scanner Beam
		float MaxRange;
		MapBuilder map_builder;


		/**
		 *	@brief Applies the logit function to a given variable
		 *	
		 *	@param x The variable to input in the logit function.
		 *
		 *	@return float - Log Odds Value 
		 */
		inline float LogOdds(float x);

		
		/**
		 * @brief An inverse measurement model for a range scanner. Looks through each
		 * 			beam in a scan and uses them to determine whether a given cell is 
		 * 			occupied or not.
		 *
		 * @param cell Current Cell under examination
		 * @param beams Range Scan
		 *
		 * @return float - Return probability value for given cell.
		 */
		float InverseSensorModel(ogrid::Cell cell, std::vector<VectorXf> beams);


		/**
		 * @brief Returns the index of the beam in a scan that is closest in heading to a given bearing.
		 *
		 * @param beams Range Scan
		 * @param cell 
		 * @param call_range 
		 * @param cell_bearing 
		 * @return int - Index of beam with a bearing most similar to the bearing parameter
		 */
		int Get_MostSimilarBeam(std::vector<VectorXf> beams, ogrid::Cell cell, float cell_range, float cell_bearing);


	public:

		/**
		 * @brief Default Constructoe
		 * 
		 */
		OccupancyGridMap();		

		/**
		 * @brief Initialize an Occupancy Grid Map
		 *
		 * @param m Number of Map Rows
		 * @param n Number of Map Columns
		 * @param alpha Width of Cell
		 * @param beta Angular width of Scanner Beam
		 * @param max_range Max Scan Range
		 */
		OccupancyGridMap(int m, int n, float alpha, float beta, float max_range);



		/**
		 * @brief Runs the Occupancy Grid Map algorithm.
		 *
		 * @param pose Current Robot Pose
		 * @param scan Range Scan
		 *
		 * @return Eigen::Tensor<float, 2> - The updated map
		 */
		Eigen::Tensor<float, 2> UpdateGridMap(VectorXf pose, std::vector<VectorXf> scan);


		/**
		 * @brief 
		 * 
		 * @param cloud 
		 * @return Eigen::Tensor<float, 2> 
		 */
		Eigen::Tensor<float, 2> UpdateGridMapWithPointCloud(PointCloud cloud);

};







