#include <iostream>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <utility>
 
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
		 * @brief 
		 *
		 * @param cell
		 * @param pose
		 * @param measurements
		 *
		 * @return float - 
		 * **/
		float InverseSensorModel(pair<int, int> cell, VectorXf pose, vector<VectorXf> measurements);



		/**
		 *	@brief
		 *	
		 *	@param x
		 *
		 *	@return float - Log Odds Value 
		 * **/
		float LogOdds(float x);



		/**
		 * @brief
		 *
		 * @param measurements
		 * @param bearing
		 *
		 * @return int - 
		 * **/
		int Get_MinIndex(vector<VectorXf> measurements, float bearing);


	public:


		/**
		 * @brief
		 *
		 * @param m
		 * @param n
		 * @param alpha
		 * @param beta
		 * @param max_range
		 *
		 * **/
		OccupancyGridMap(int m, int n, float alpha, float beta, float max_range);



		/**
		 *
		 * @brief
		 *
		 * @param pose
		 * @param measurements
		 *
		 * @return void - 
		 *
		 * **/
		void UpdateGridMap(VectorXf pose, vector<VectorXf> measurements);

};







