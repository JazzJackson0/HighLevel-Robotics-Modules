#include "RRT.hpp"

// Private-------------------------------------------------------------------------------------------------------------------------------------

bool RRT::isValid(int x, int y) {
    return (x >= 0) && (x < WIDTH) && 
        (y >= 0) && (y < HEIGHT);
}

bool RRT::isBlocked(int x, int y) {

    int row = y;
    int col = x;
    return MAP(row, col) != 0.f;
}


bool RRT::isGoalReached(int x, int y) {

    return (x == Goal.x && y == Goal.y);
}


bool RRT::isStartAndGoalValid() {
    
    if (!isValid(Start.x, Start.y)) {
        std::cout << "Invalid Starting Coordinates." << std::endl;
        return false;
    }

    if (!isValid(Goal.x, Goal.y)) {
        std::cout << "Invalid Goal Coordinates." << std::endl;
        return false;
    }

    if (isBlocked(Start.x, Start.y)) {
        std::cout << "Start Coodrdinates Blocked." << std::endl;
        return false;
    }

    if (isGoalReached(Start.x, Start.y)) {
        std::cout << "Ummm... You're already there..." << std::endl;
        return false;
    }

    return true;
}


bool RRT::isVisible(RRT_Node node, RRT_Node nearest) {
    
    // Bresenham's line algorithm for line of sight checking
    int dx = std::abs(node.x - nearest.x);
    int dy = std::abs(node.y - nearest.y);
    int sx = (nearest.x < node.x) ? 1 : -1;
    int sy = (nearest.y < node.y) ? 1 : -1;
    int err = dx - dy;

    int x_curr = nearest.x;
    int y_curr = nearest.y;

    while (true) {
		// Clear line of sight
        if (x_curr == node.x && y_curr == node.y) { return true; }

		// Line of sight blocked by obstacle
        if (isBlocked(x_curr, y_curr) == 1.0) { return false; }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x_curr += sx;
        }
        if (e2 < dx) {
            err += dx;
            y_curr += sy;
        }
    }
}


float RRT::Get_Distance(RRT_Node node_a, RRT_Node node_b) {

	return (sqrt( pow( ( node_a.x - node_b.x ), 2 ) + 
                pow( ( node_a.y - node_b.y ), 2 ) ));
}


RRT_Node RRT::Get_RandomPosition() {
    
	std::uniform_int_distribution<int> height_dist(0, HEIGHT - 1);
    std::uniform_int_distribution<int> width_dist(0, WIDTH - 1);

	int x = width_dist(rd);
    int y = height_dist(rd);
	return RRT_Node(x, y);
}


bool RRT::Get_Neighbors(float search_radius, RRT_Node randPos, std::vector<int> &neighbors) {

    int n_Vertices = RapidTree.Get_NumOfVertices();

    // Calculate distance between randPos and every vertex in set.
    for (int i = 0; i < n_Vertices; i++) {

		float dist = Get_Distance(randPos, RapidTree.Get_Vertex(i));

        if (dist <= SearchRadius) { neighbors.push_back(i); }
    }

	return neighbors.size() > 0;
}


std::pair<int, float> RRT::Get_BestNeighbor(std::vector<int> neighbors, RRT_Node randPos) {

	float min_cost = std::numeric_limits<float>::max();
	int best_index = -1;
	for (int i = 0; i < neighbors.size(); i++) {

		float cost = RapidTree.Get_Vertex(neighbors[i]).dist_from_start;

		if (cost < min_cost) { 
			min_cost = cost;
			best_index = i;
		}
	}

	float dist = Get_Distance(randPos, RapidTree.Get_Vertex(best_index));
	std::pair<int, float> best_neighbor = make_pair(best_index, dist);
	return best_neighbor;
}


std::pair<int, float> RRT::Get_NearestVertexIndex(RRT_Node randPos) {
    
    float min_dist = std::numeric_limits<float>::max();
    int nearestVertexIndex = -1;
    int n_Vertices = RapidTree.Get_NumOfVertices();

    // Calculate distance between randPos and every vertex in set.
    for (int i = 0; i < n_Vertices; i++) {

		RRT_Node node_i = RapidTree.Get_Vertex(i);

		// This random position already exists
		if (randPos.x == node_i.x && randPos.y == node_i.y) {
			repeat_node = true;
			return make_pair(-1, 0);  
		}

		float dist = Get_Distance(randPos, node_i);

        if (dist < min_dist) { 
            min_dist = dist;
            nearestVertexIndex = i;
        }
    }
    return make_pair(nearestVertexIndex, min_dist);  
}


bool RRT::Move_NodeCloser(RRT_Node &node, RRT_Node nearest) {

	Eigen::VectorXi v(2);
	v << (node.x - nearest.x), (node.y - nearest.y);
	float v_magnitude = std::sqrt((v[0] * v[0]) + (v[1] * v[1]));
	VectorXf unit_vec(2);
	unit_vec << round(v[0] / v_magnitude), round(v[1] / v_magnitude);

	node.x = nearest.x + (int)(MaxConnectionDistance * unit_vec[0]);
	node.y = nearest.y + (int)(MaxConnectionDistance * unit_vec[1]);

	return isBlocked(node.x, node.y);
}


bool RRT::Connect_NewVertex(std::pair<int, float> nearest_info, RRT_Node new_node) {

	float new_to_nearest = nearest_info.second;
	int nearest_id = nearest_info.first;
	RRT_Node nearest = RapidTree.Get_Vertex(nearest_id);
	RRT_Edge null_edge;
	
    // [OPTION 1] Location is blocked--------------------------------------------------------
    if (isBlocked(new_node.x, new_node.y)) { return false; }

    // [OPTION 2] Random coordinate is <= Max Connect Distance: -----------------------------
    if (new_to_nearest <= MaxConnectionDistance) {
		if(!isVisible(new_node, nearest)) { return false; }

        // [Add new node to Graph]
		new_node.dist_from_start = new_to_nearest + nearest.dist_from_start;
		new_node.parent_idx = nearest_id;
		RapidTree.Add_Vertex(new_node, false, null_edge);

		// [Add new edge to Graph]
		RRT_Edge edge = RRT_Edge(new_to_nearest, new_node.parent_idx, RapidTree.Get_NumOfVertices() - 1);
		RapidTree.Add_Edge(edge.child_index, edge.parent_index, edge);
        return true;
    }

    // [OPTION 3] Random coordinate is farther than Max Connect Distance:--------------------
	if(!Move_NodeCloser(new_node, nearest)) { return false; }
	if(!isVisible(new_node, nearest)) { return false; }

	new_to_nearest = Get_Distance(new_node, nearest);

	// [Add new node to Graph]
	new_node.dist_from_start = new_to_nearest + nearest.dist_from_start;
	new_node.parent_idx = nearest_id;
	RapidTree.Add_Vertex(new_node, false, null_edge);

	// [Add new edge to Graph]
	RRT_Edge edge = RRT_Edge(new_to_nearest, new_node.parent_idx, RapidTree.Get_NumOfVertices() - 1);	
	RapidTree.Add_Edge(edge.child_index, edge.parent_index, edge);
	return true;
}


void RRT::Rewire_Neighbors(std::vector<int> neighbors, int newest_node_idx) {

	RRT_Node new_node = RapidTree.Get_Vertex(newest_node_idx);
	for (int i = 0; i < neighbors.size(); i++) {

		RRT_Node neighbor = RapidTree.Get_Vertex(neighbors[i]);
		float new_to_nearest = Get_Distance(new_node, neighbor);
		float new_dist_from_start = new_node.dist_from_start + new_to_nearest;

		// If neighbor gets a shorter path to Start by connecting to new node.
		if (new_dist_from_start < neighbor.dist_from_start && isVisible(new_node, neighbor)) {
			
			// [Cancel Old Connection]
			RapidTree.Remove_Edge(neighbor.parent_idx, neighbors[i]);
			neighbor.parent_idx = newest_node_idx;
			RapidTree.Update_VertexData(neighbors[i], neighbor);

			// [Create New Connection]
			RRT_Edge edge = RRT_Edge(new_to_nearest, newest_node_idx, neighbors[i]);
			RapidTree.Add_Edge(edge.child_index, edge.parent_index, edge);
			
			// Recalculate the 'dist_from_start' for all children of edge.child_index.
			Update_Distances(edge.child_index);
		}	
	}
}


void RRT::Update_Distances(int node_index) {

	//std::cout << "Node Index: " << node_index << std::endl;
	RRT_Node parent_node = RapidTree.Get_Vertex(node_index);
	std::vector<RRT_Edge> edges = RapidTree.Get_IncidentEdges(node_index);
	
	for (int i = 0; i < edges.size(); i++) {
	
		// Update Child Node's Distance from Start
		if (edges[i].parent_index == node_index) {

			RRT_Node child_node = RapidTree.Get_Vertex(edges[i].child_index);
			child_node.dist_from_start = parent_node.dist_from_start + edges[i].distance;
			RapidTree.Update_VertexData(edges[i].child_index, child_node);
			Update_Distances(edges[i].child_index);
		}
	}

	return;
}



std::stack<VectorXi> RRT::PathTraceHelper(int goal_node_idx, int current_node_idx, std::stack<VectorXi> path) {

	RRT_Node current_node = RapidTree.Get_Vertex(current_node_idx);
	VectorXi waypoint(2);
	waypoint << current_node.x, current_node.y;
	path.push(waypoint);

	if (current_node_idx == goal_node_idx) {
		return path;
	}

	std::vector<RRT_Edge> edges = RapidTree.Get_IncidentEdges(current_node_idx);
	for (int i = 0; i < edges.size(); i++) {

		// Move up to Parent Node
		if (edges[i].child_index == current_node_idx) {
			return PathTraceHelper(goal_node_idx, edges[i].parent_index, path);
		}
	}
	return path;
}




std::vector<VectorXi> RRT::PathTrace(int goal_node_idx, int current_node_idx) {

	std::stack<VectorXi> path_in;
	std::stack<VectorXi> path_out = PathTraceHelper(goal_node_idx, current_node_idx, path_in);
	std::vector<VectorXi> waypoints;

	int path_size = path_out.size();
	for (int i = 0; i < path_size; i++) {
		waypoints.push_back(path_out.top());
		path_out.pop();
	}

	return waypoints;
}


// Public-------------------------------------------------------------------------------------------------------------------------------------

RRT::RRT() { /*Default Constructor*/ }


RRT::RRT(Eigen::Tensor<float, 2> map) : MAP(map) { 

	auto &d = MAP.dimensions();
	WIDTH = d[1];
	HEIGHT = d[0];
	// std::cout << std::endl;
    // std::cout << "Map:" << std::endl;
    // std::cout << MAP << std::endl;
    // std::cout << std::endl;
}



void RRT::Load_MAP(Eigen::Tensor<float, 2> map) {

    MAP = map;

    auto &d = MAP.dimensions();
	WIDTH = d[1];
	HEIGHT = d[0];
    // std::cout << std::endl;
    // std::cout << "Map:" << std::endl;
    // std::cout << MAP << std::endl;
    // std::cout << std::endl;
}



std::vector<VectorXi> RRT::RRT_Path(VectorXi start, VectorXi goal, float maxConnectionDistance) {
	
	MaxConnectionDistance = maxConnectionDistance;
	RRT_Edge null_edge;
	Start.x = start[0];
	Start.y = start[1];
	Start.dist_from_start = 0.f;
	Goal.x = goal[0];
	Goal.y = goal[1];
	if (!isStartAndGoalValid()) {
		VectorXi start_pt(2);
		start_pt << Start.x, Start.y;
		std::vector<VectorXi> invalid_path;
		invalid_path.push_back(start_pt);
		return invalid_path;
	}

    RapidTree.Add_Vertex(Start, false, null_edge);
    int recently_added = RapidTree.Get_NumOfVertices() - 1;
    
    // While the Goal has not been reached.
    while (!(RapidTree.Get_Vertex(recently_added).x == Goal.x && RapidTree.Get_Vertex(recently_added).y == Goal.y)) {

		repeat_node = false;
        RRT_Node randP = Get_RandomPosition();
		std::pair<int, float> nearestVertex = Get_NearestVertexIndex(randP);
		if (repeat_node) { continue; }
        if (!Connect_NewVertex(nearestVertex, randP)) { continue; }
		Connect_NewVertex(nearestVertex, randP);
		recently_added = RapidTree.Get_NumOfVertices() - 1;

		// std::cout << "Added to Tree (" << RapidTree.Get_Vertex(recently_added).x << ", " << RapidTree.Get_Vertex(recently_added).y << ")" << std::endl;
		// std::cout << "Tree Size: " << RapidTree.Get_NumOfVertices() << std::endl;
    }

	return PathTrace(0, RapidTree.Get_NumOfVertices() - 1);
}



std::vector<VectorXi> RRT::RRTStar_Path(VectorXi start, VectorXi goal, float maxConnectionDistance, float search_radius) {
	
	MaxConnectionDistance = maxConnectionDistance;
	RRT_Edge null_edge;
	Start.x = start[0];
	Start.y = start[1];
	Start.dist_from_start = 0.f;
	Goal.x = goal[0];
	Goal.y = goal[1];
	SearchRadius = search_radius;
	if (!isStartAndGoalValid()) {
		VectorXi start_pt(2);
		start_pt << Start.x, Start.y;
		std::vector<VectorXi> invalid_path;
		invalid_path.push_back(start_pt);
		return invalid_path;
	}

	RapidTree.Add_Vertex(Start, false, null_edge);
    int recently_added = RapidTree.Get_NumOfVertices() - 1;
    
    // While the Goal has not been reached.
    while (!(RapidTree.Get_Vertex(recently_added).x == Goal.x && RapidTree.Get_Vertex(recently_added).y == Goal.y) ) {
		
		repeat_node = false;
		std::vector<int> neighbors;
        RRT_Node randP = Get_RandomPosition();
		std::pair<int, float> nearest_index = Get_NearestVertexIndex(randP);
		if (repeat_node) { continue; }
		if (!Get_Neighbors(SearchRadius, randP, neighbors)) { continue; }
		std::pair<int, float> best_index_in_radius = Get_BestNeighbor(neighbors, randP);
		
		if (best_index_in_radius.second <= nearest_index.second) {
			if (!Connect_NewVertex(best_index_in_radius, randP)) { continue; }
		}

		else { 
			if (!Connect_NewVertex(nearest_index, randP)) { continue; }
		}

		// Re-wire other neighbors in search radius
		int randP_index = RapidTree.Get_NumOfVertices() - 1;
		Rewire_Neighbors(neighbors, randP_index);

		recently_added = randP_index;

		// std::cout << "Added to Tree (" << RapidTree.Get_Vertex(recently_added).x << ", " << RapidTree.Get_Vertex(recently_added).y << ")" << std::endl;
		// std::cout << "Tree Size: " << RapidTree.Get_NumOfVertices() << std::endl;
    }

	return PathTrace(0, RapidTree.Get_NumOfVertices() - 1);
}




/*
 * 			TO-DO
 * 			-----
 *  - Incorporate Costmap to keep robot from hugging the edges
 *
 *  - 
 *
 *  - 
 *  */
