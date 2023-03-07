#include "BUG.hpp"

DistSensorData Bug::Get_DistSensorData() {
	


	// Receive Distance Data from Serial Comm

}



Pose Bug::Get_CurrentPosition() {
	// Receive Odometry Data from Serial Comm
}



vector<vector<float>> Bug::Get_RangeScanData() {

	// Receive Scan data from ... somewhere
}



void Bug::Go(Directions direction, float speed) {

	if (direction == FORWARD) { /*Send [Travel Direction] & [Speed] over Serial Comm*/ }
	else if (direction == LEFT) { /*Send [Travel Direction] & [Speed] over Serial Comm*/ }
	else if (direction == RIGHT) { /*Send [Travel Direction] & [Speed] over Serial Comm*/ }
	else if (direction == STOP) { /*Send [Travel Direction] & [Speed] over Serial Comm*/ }
}



float Bug::Get_GoalDistance(CellCoordinate currentPos) {
	return (float)(sqrt( pow( ( currentPos.first - goalPoint.first ), 2 ) + 
		pow( ( currentPos.second - goalPoint.second ), 2 ) ));
}



float Bug::Get_GoalDirection() {

	int X = abs( startPoint.first - goalPoint.first );
	int Y = abs( startPoint.second - goalPoint.second );
	return atan2(Y, X) - currentOrientation;
}

		

void Bug::TurnInDirection(float desired_angle) {
	
	while (currentOrientation < desired_angle) { 

		Go(RIGHT, turn_speed);
		currentOrientation = Get_CurrentPosition().orientation;
	}

	while (currentOrientation > desired_angle) { 

		Go(LEFT, turn_speed);
		currentOrientation = Get_CurrentPosition().orientation;
	}
}



void Bug::TurnLeft() {
	TurnInDirection(90);	
}

	

void Bug::TurnCorner(float right_sensor) {

	// Move robot forward for a sec or so, so it clears the wall and can make a clean turn.

	// Turn
	TurnInDirection(-90);
	
	// Then Move forward until the right sensor picks up the wall again.
	while (right_sensor > right_dist) { Go(FORWARD, forward_speed); }	
}



void Bug::TurnInnerCorner() {
	
	// Move robot forward for a sec or so, so its right sensor still picks up wall when turned.

	// Turn
	TurnLeft();
}



void Bug::UpdateClosestPointToGoal() {
	
	float dist = Get_GoalDistance(Get_CurrentPosition().position);
	if (dist < min_dist) { 
			
		min_dist = dist; 
		leave_point = Get_CurrentPosition().position;
	}
}



void Bug::UpdateCurrentAngleToGoal() {
	
	currentAngleToGoal = Get_GoalDirection();
}



int Bug::Get_BeamClosestToGoal(vector<vector<float>> scan) {

	float min_beam = std::numeric_limits<float>::max();
	int best_index = -1;
	for (int i = 0; i < scan.size(); i++) {
		
		int x = (int) scan[i][0] * cos(scan[i][1]);
		int y = (int) scan[i][0] * sin(scan[i][1]);
		float dist = Get_GoalDistance(make_pair(x, y));
		if (dist < min_beam) {
			
			min_beam = dist;
			best_index = i;
		}
	}

	return best_index;
}



bool Bug::CheckForMLineHit() {
	
	return currentAngleToGoal == m_line_angle;
}



void Bug::FollowWall(BugType type) {
	
	
	while (1) {
		
		DistSensorData currentData = Get_DistSensorData();
		vector<vector<float>> currentScan = Get_RangeScanData();

		if (type == BUG1 || type == TAN) { 
		
			// If robot returns to the initial point of entry, then it has circumnavigated the whole obstacle
			if (Get_CurrentPosition().position == entry_point && !(entering_wallfollow_mode) ) {
			
				leave_at_leavepoint = true;
				
				// There is no way to reach goal
				if (TAN) {
					return;
				}
			}
		}	
	
		// 1) If Wall is detected on right and not in front. -> GO FORWARD
		if (currentData.front_dist > forward_dist && currentData.right_dist <= right_dist) {
			
			Go(FORWARD, forward_speed); // Add a time element to Go?!!!
		}
	
		// 2) If at Inner Corner. -> TURN INNER CORNER
		else if (currentData.front_dist <= forward_dist && currentData.right_dist <= right_dist) {
			
			// If within desired range for a clean turn
			TurnInnerCorner();

			// Else: move up a little more
			
		}

		// 3) If no Wall is present on right or front, then you are at a corner. -> TURN CORNER
		else if (currentData.front_dist > forward_dist && currentData.right_dist > right_dist) { 
		
			if (type == BUG0) {
				return;
			}

			TurnCorner(-90); 
		}

		// 4) If Wall is detected only in front, then turn to follow new wall -> TURN LEFT.
		else if (currentData.front_dist <= forward_dist && currentData.right_dist > right_dist) {
			
			if (entering_wallfollow_mode) {
				entry_point = Get_CurrentPosition().position;
				entering_wallfollow_mode = false;
			}

			if (TAN) {

				int index = Get_BeamClosestToGoal(currentScan);
				firstMinScanPoint.first = (int) currentScan[index][0] * cos(currentScan[index][1]);
				firstMinScanPoint.second = (int) currentScan[index][0] * sin(currentScan[index][1]);

				if (Get_GoalDistance(left_scanPoint) <= Get_GoalDistance(right_scanPoint)) { TurnLeft(); }

				else { TurnInDirection(-90); }
			}
			

			// If within desired range for a clean turn
			TurnLeft();
		
			if (entering_wallfollow_mode) {
				entry_point = Get_CurrentPosition().position;
				entering_wallfollow_mode = false;
			}
			
			// Else: Move up a little more
		}	

		if (type == BUG1) { 
		
			UpdateClosestPointToGoal();

			if (leave_at_leavepoint && Get_CurrentPosition().position == leave_point) { // Rough idea
			
				leave_at_leavepoint = false;
				return;
			}

		}

		else if (type == BUG2) {
		
			UpdateCurrentAngleToGoal();

			if (CheckForMLineHit()) {
			
				return;
			}
		}
		
		else if (type == TAN) {

			int index = Get_BeamClosestToGoal(currentScan);
			int x = currentScan[index][0] * cos(currentScan[index][1]);
			int y = currentScan[index][0] * sin(currentScan[index][1]);

			if (Get_GoalDistance(make_pair(x, y)) < Get_GoalDistance(firstMinScanPoint)) {
				
				// Leave the obstacle boundary
				return;
			}
		}

		// If goal is reached during Wall Following, exit Wall Following Mode
		if ( currentPos.first == goalPoint.first && currentPos.second == goalPoint.second) { return; }

	}
}



Bug::Bug(Pose start_pose, CellCoordinate goal, float f_speed, float t_speed) {
	startPoint.first = start_pose.position.first;
	startPoint.second = start_pose.position.second;
	startOrientation = start_pose.orientation;
	goalPoint.first = goal.first;
	goalPoint.second = goal.second;
	min_dist = std::numeric_limits<float>::max(); 
	forward_speed = f_speed;
	turn_speed = t_speed;
}



void Bug::Bug0() {
	
	TurnInDirection( Get_GoalDirection() );

	// While not at goal, Go to Goal
	while ( !(Get_CurrentPosition().position.first == goalPoint.first && 
		Get_CurrentPosition().position.second == goalPoint.second) ) {
		
		Go(FORWARD, forward_speed);
		
		if (Get_DistSensorData().front_dist <= forward_dist) { 
			
			entering_wallfollow_mode = true;
			FollowWall(BUG0);
			TurnInDirection( Get_GoalDirection() );
		}
	}
}



void Bug::Bug1() {
	
	TurnInDirection( Get_GoalDirection() );

	// While not at goal, Go to Goal
	while ( !(Get_CurrentPosition().position.first == goalPoint.first && 
		Get_CurrentPosition().position.second == goalPoint.second) ) {
		
		Go(FORWARD, forward_speed);
		
		if (Get_DistSensorData().front_dist <= forward_dist) { 
			
			entering_wallfollow_mode = true;
			FollowWall(BUG1);
			TurnInDirection( Get_GoalDirection() );
		}
	}
}



void Bug::Bug2() {
	
	m_line_angle = Get_GoalDirection();
	TurnInDirection( m_line_angle );

	// While not at goal, Go to Goal
	while ( !(Get_CurrentPosition().position.first == goalPoint.first && 
		Get_CurrentPosition().position.second == goalPoint.second) ) {
		
		Go(FORWARD, forward_speed);
		
		if (Get_DistSensorData().front_dist <= forward_dist) { 
			
			entering_wallfollow_mode = true;
			FollowWall(BUG2);
			TurnInDirection( m_line_angle );
		}
	}
}



void Bug::TangentBug() {
	
	vector<vector<float>> range_scan = Get_RangeScanData();
	TurnInDirection( Get_GoalDirection() );
	left_scanPoint.first = range_scan.front()[0] * cos(range_scan.front()[1]);
	left_scanPoint.second = range_scan.front()[0] * sin(range_scan.front()[1]);
	right_scanPoint.first = range_scan.back()[0] * cos(range_scan.back()[1]);
	right_scanPoint.second = range_scan.back()[0] * sin(range_scan.back()[1]);

	// While not at goal, Go to Goal
	while ( !(Get_CurrentPosition().position.first == goalPoint.first && 
		Get_CurrentPosition().position.second == goalPoint.second) ) {
		
		Go(FORWARD, forward_speed);
		
		if (Get_DistSensorData().front_dist <= forward_dist) { 
			
			entering_wallfollow_mode = true;
			FollowWall(TAN);
			TurnInDirection( Get_GoalDirection() );
		}
	}
}



/*
 * 			TO-DO
 * 			-----
 *  - Complete the Class
 *
 *  - Add access to Beaglebone Serial Comm API
 *
 *  - Look at BUG2 psedocode. I'm missing something important about m_line direction.
 *  
 *  - Test Code 
 *  */
