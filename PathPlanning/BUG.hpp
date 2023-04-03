#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <limits>
using std::pair;
using std::make_pair;
using std::vector;

typedef enum { FORWARD, LEFT, RIGHT, STOP } Directions;
typedef enum { BUG0, BUG1, BUG2, TAN } BugType;
typedef pair<int, int> CellCoordinate;

struct Pose {
	CellCoordinate position;
	float orientation;
};

struct DistSensorData {
	float front_dist;
	float right_dist;
	float left_dist;	
};


class Bug {
	
    private:
        
		float right_dist; // The viable detection range for right distance sensor
		float forward_dist; // The viable detection range for front distance sensor
        float forward_speed; // 1% - 100%
        float turn_speed; // 1% - 100%

		CellCoordinate startPoint;
		float startOrientation;
        CellCoordinate goalPoint;

		float currentOrientation;
		float currentAngleToGoal;
		CellCoordinate currentPos;

		float min_dist; // Used to help calculate the leave_point
		CellCoordinate entry_point; // Point where robot begins orbiting obstacle
		CellCoordinate leave_point; // Closest point to goal on the orbit of given obstacle
		float angle_to_goal; // Current angle to goal based on robot's position
		float m_line_angle;
		CellCoordinate right_scanPoint;
		CellCoordinate left_scanPoint;
		CellCoordinate firstMinScanPoint; // Point, from first new obstacle scan, closest to goal.

		bool entering_wallfollow_mode = false;
		bool leave_at_leavepoint = false; // Enables robot to leave obstacle orbit when it reaches predefined closest_point.

		
		/**
		 * @brief Get current distance sensor scan 
		 *
		 * @return ** DistSensorData
		 */
		DistSensorData Get_DistSensorData();



		/**
         * @brief Get vehicle's current position.
         * 
         * @return ** Pose - Robot's current pose information (position & orientation)
         */
        Pose Get_CurrentPosition();


		/**
		 * @brief Get current range scan  
		 *
		 * @return ** vector<vector<float>> - DistSensorData
		 */
		vector<vector<float>> Get_RangeScanData();



		/**
         * @brief Move vehicle in a given direction at a given speed.
         * 
         * @param direction Direction to travel
         * @param speed Speed of travel
         * @return ** void 
         */
        void Go(Directions direction, float speed);



        /**
         * @brief Get distance from a given current (x, y) position to goal (x, y) position
         * 
         * @param currentPos Current position
         * @return ** float - Distance to Goal 
         */
        float Get_GoalDistance(CellCoordinate currentPos);



        /**
         * @brief Get direction (angle) of goal relative to the 
         *          vehicle's current orientation.
		 *
         * @return ** float - Direction (angle) of Goal 
         */
        float Get_GoalDirection();

        		
		
		/**
		 * @breif Turn robot until it is in line with a given angle
		 * 			relative to itself. 
		 *
		 * @param desired_angle The angle to align with
		 *
		 * @return ** void 
		 */
		void TurnInDirection(float desired_angle);



		/**
		 * @brief Turns the robot left.
		 *
		 * @return ** void
		 */
		void TurnLeft();


			
		/**
		 * @brief Moves robot around corner when the front & right distance sensors 
		 * 			return value greater than a predefined distance. 
		 *
		 * @param Right sensor distance reading
		 *
		 * @return ** void
		 */
		void TurnCorner(float right_sensor);



		/**
		 * @brief Turns robot 90 degress to get out of inner corner.
		 *
		 * @return ** void
		 */
		void TurnInnerCorner();


		/**
		 * @brief (Used while in Follow Wall Mode for BUG 1) Calculate the distance to goal from the current 
		 * 			position and update the (x, y) leave_point variable if the current position is the 
		 * 			closest to the goal.
		 *
		 * @return ** void
		 */
		void UpdateClosestPointToGoal();


		/**
		 * @brief Updates the angle from the current position to goal.
		 *
		 * @return ** void
		 */
		void UpdateCurrentAngleToGoal();


		/**
		 * @brief Get the index of the beam closest to goal by calculating the heuristic distance function.
		 * 
		 * @param scan Range Scan
		 *
		 * @return ** int - Index of beam closest to goal point.
		 */
		int Get_BeamClosestToGoal(vector<vector<float>> scan);


		/**
		 * @brief Checks if current position intersects wiith the m-line.
		 *
		 * @return ** void
		 */
		bool CheckForMLineHit();



		/**
		 * @brief Follows the boundary of a wall
		 *
		 * @param type Bug Algorithm Type
		 *
		 * @return ** void
		 */
		void FollowWall(BugType type);


    public:

		/**
		 * @brief Initializes a Bug Algorithm object
		 *
		 * @param start_pose - The starting pose
		 * @param goal - The goal point
		 * @param f_speed The desired forward speed
		 * @param t_speed The desired turn speed
		 *
		 */
        Bug(Pose start_pose, CellCoordinate goal, float f_speed, float t_speed);


        /**
		 * @brief Runs the BUG 0 algorithm
		 *
		 * @return ** void
		 */
		void Bug0();


		 /**
		 * @brief Runs the BUG 1 algorithm
		 *
		 * @return ** void
		 */
        void Bug1();


		 /**
		 * @brief Runs the BUG 2 algorithm
		 *
		 * @return ** void
		 */
        void Bug2();


		 /**
		 * @brief Runs the Tangent BUG algorithm
		 *
		 * @return ** void
		 */
        void TangentBug();

};


