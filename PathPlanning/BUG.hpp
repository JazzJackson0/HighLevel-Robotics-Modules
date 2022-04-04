#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <limits>
using std::pair;
using std::make_pair;
using std::vector;

typedef struct pose Pose;
typedef struct dist_sensor_data DistSensorData;
typedef enum { FORWARD, LEFT, RIGHT, STOP } Directions;
typedef enum { BUG0, BUG1, BUG2, TAN } BugType;


class Bug {
	
    private:
        
		float right_dist; // The viable detection range for right distance sensor
		float forward_dist; // The viable detection range for front distance sensor
        float forward_speed; // 1% - 100%
        float turn_speed; // 1% - 100%

		pair<int, int> startPoint;
		float startOrientation;
        pair<int, int> goalPoint;

		float currentOrientation;
		float currentAngleToGoal;
		pair<int, int> currentPos;

		float min_dist; // Used to help calculate the leave_point
		pair<int, int> entry_point; // Point where robot begins orbiting obstacle
		pair<int, int> leave_point; // Closest point to goal on the orbit of given obstacle
		float angle_to_goal; // Current angle to goal based on robot's position
		float m_line_angle;
		pair<int, int> right_scanPoint;
		pair<int, int> left_scanPoint;
		pair<int, int> firstMinScanPoint; // Point, from first new obstacle scan, closest to goal.

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
		 * @return ** DistSensorData
		 */
		vector<float> Get_RangeScanData();



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
        float Get_GoalDistance(pair<int, int> currentPos);



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
		 * @brief 
		 *
		 * @return ** void
		 */
		void UpdateCurrentAngleToGoal();


		/**
		 * @brief Get the index of the beam closest to goal by calculating the heuristic distance function.
		 *
		 * @return ** int - Index of beam closest to goal point.
		 */
		int Get_BeamClosestToGoal(vector<float> scan);


		/**
		 * @brief 
		 *
		 * @return ** void
		 */
		bool CheckForMLineHit();



		/**
		 * @brief 
		 *
		 * @param type Bug Algorithm Type
		 *
		 * @return ** void
		 */
		void FollowWall(BugType type);


    public:

		/**
		 * @brief 
		 *
		 * @param start_pose - The starting pose
		 * @param goal - The goal point
		 * @param f_speed The desired forward speed
		 * @param t_speed The desired turn speed
		 *
		 */
        Bug(Pose start_pose, pair<int, int> goal, float f_speed, float t_speed);


        /**
		 * @brief 
		 *
		 * return ** void
		 */
		void Bug0();


		 /**
		 * @brief 
		 *
		 * return ** void
		 */
        void Bug1();


		 /**
		 * @brief 
		 *
		 * return ** void
		 */
        void Bug2();


		 /**
		 * @brief 
		 *
		 * @param
		 *
		 * return ** void
		 */
        void TangentBug();

};



struct pose {
	pair<int, int> position;
	float orientation;
};

struct dist_sensor_data {
	float front_dist;
	float right_dist;
	float left_dist;	
};

