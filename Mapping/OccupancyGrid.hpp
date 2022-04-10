#include <iostream>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <utility>
#include <vector>
using namespace::Eigen;
using std::vector;
using std::pair;
using std::make_pair;

class OccupancyGridMap {

	private:
		MatrixXf InitialGridMap;
		MatrixXf GridMap;
		MatrixXf PreviousGridMap;
		int M;
		int N;
		float Alpha;
		float Beta;
		float MaxRange;

		/**
		 * @brief An inverse measurement model for a range scanner. Looks through each
		 * 			beam in a scan and uses them to determine whether a given cell is 
		 * 			occupied or not.
		 *
		 * @param cell Current Cell under examination
		 * @param pose Current pose of the robot
		 * @param scan Range Scan
		 *
		 * @return float - Return probability value for given cell.
		 */
		float InverseSensorModel(pair<int, int> cell, VectorXf pose, vector<VectorXf> scan);



		/**
		 *	@brief Applies the logit function to a given variable
		 *	
		 *	@param x The variable to input in the logit function.
		 *
		 *	@return float - Log Odds Value 
		 */
		float LogOdds(float x);



		/**
		 * @brief Returns the index of the beam in a scan that is closest to a given bearing.
		 *
		 * @param scan Range Scan
		 * @param bearing The bearing to compare each beam to.
		 *
		 * @return int - Index of beam with a bearing most similar to the bearing parameter
		 */
		int Get_MinIndex(vector<VectorXf> scan, float bearing);


	public:


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
		 * @return void 
		 */
		void UpdateGridMap(VectorXf pose, vector<VectorXf> scan);

};







