#include "RRT.hpp"

std::pair<int, float> RRT::Get_NearestVertexIndex(Node randPos) {
    
    float min_dist = std::numeric_limits<float>::max();
    int nearestVertexIndex = -1;
    int n_Vertices = RapidTree.Get_NumOfVertices(); // Number of Vertices in Graph

    // Calculate distance between randPos and every vertex in set.
    for (int i = 0; i < n_Vertices; i++) {
        
		float dist = Get_Distance(randPos, RapidTree.Get_Vertex(i));

        if (dist < min_dist) { 
            min_dist = dist;
            nearestVertexIndex = i;
        }
    }

    return make_pair(nearestVertexIndex, min_dist);  
}



Node RRT::Get_RandomPosition() {
    
    srand(time(NULL));
	Node randNode;
    int x = rand() % G_HEIGHT;
    int y = rand() % G_WIDTH;
	randNode.x = x;
	randNode.y = y;
    return randNode;
}



bool RRT::Set_NewVertex(std::pair<int, float> nearestVIndex, Node randPos, 
    int maxConnectDist) {

	Node nearest = RapidTree.Get_Vertex(nearestVIndex.first);
	float prev_dist_from_start = nearest.DistanceFromStart;

    // If location is blocked
    if (Grid[randPos.x][randPos.y] == 1) { return false; }

    /* If random coordinate is closer than Max Connect Distance, 
    then just place new Vertx at random coordinate. */
    if (nearestVIndex.second < maxConnectDist) {
        
        // Turn random position into a vertex and add it to Graph
		randPos.DistanceFromStart = Get_Distance(randPos, nearest) + prev_dist_from_start;
		randPos.ParentIndex = nearestVIndex.first;
		RapidTree.Add_Edge(RapidTree.Add_Vertex(randPos, false, 0).VertexID, randPos.ParentIndex, 0);
        return true;
    }

    // else: move given distance from qNear in direction of random position.
	float angle = atan2(randPos.y, randPos.x);
	Node qNew;
	qNew.x = (int)(maxConnectDist * cos(angle));
	qNew.y = (int)(maxConnectDist * sin(angle));
	qNew.DistanceFromStart = Get_Distance(qNew, nearest) + prev_dist_from_start;
	qNew.ParentIndex = nearestVIndex.first;
	RapidTree.Add_Edge(RapidTree.Add_Vertex(qNew, false, 0).VertexID, qNew.ParentIndex, 0);
	return true;
}



std::vector<int> RRT::Get_Neighbors(float search_radius, Node randPos) {

	std::vector<int> NeighboringVectors;

    int n_Vertices = RapidTree.Get_NumOfVertices(); // Number of Vertices in Graph

    // Calculate distance between randPos and every vertex in set.
    for (int i = 0; i < n_Vertices; i++) {
        
		float dist = Get_Distance(randPos, RapidTree.Get_Vertex(i));

        if (dist <= SearchRadius) { NeighboringVectors.push_back(i); }
    }

	return NeighboringVectors;
}



std::pair<int, float> RRT::Get_BestNeighbor(std::vector<int> neighbors, Node randPos) {

	float min_cost = std::numeric_limits<float>::max();
	int best_index = -1;
	for (int i = 0; i < neighbors.size(); i++) {
		
		float cost = RapidTree.Get_Vertex(neighbors[i]).DistanceFromStart;

		if (cost < min_cost) { 
			min_cost = cost;
			best_index = i;
		}
	}

	float dist = Get_Distance(randPos, RapidTree.Get_Vertex(best_index));
	std::pair<int, float> best_neighbor = make_pair(best_index, dist);
	return best_neighbor;
}



void RRT::Rewire_Neighbors(std::vector<int> neighbors, int nodeIndex, Node randPos) {

	for (int i = 0; i < neighbors.size(); i++) {

		float neighbor_dist = randPos.DistanceFromStart + Get_Distance(randPos, RapidTree.Get_Vertex(neighbors[i]));
		if (neighbor_dist < RapidTree.Get_Vertex(neighbors[i]).DistanceFromStart) {

			RapidTree.Remove_Edge(i, RapidTree.Get_Vertex(i).ParentIndex);
			RapidTree.Add_Edge(i, nodeIndex, 0);
			// You need to recalculate the DistanceFromStart for all children of i.
				// Maybe using BFS or DFS
		}	
	}
}



float RRT::Get_Distance(Node node_a, Node node_b) {
	
	return (sqrt( pow( ( node_a.x - node_b.x ), 2 ) + 
                pow( ( node_a.y - node_b.y ), 2 ) ));
}



RRT::RRT(int **grid, int width, int height) {   
    
    Grid = grid;
    G_WIDTH = width;
    G_HEIGHT = height;
}



void RRT::Run_RRT(Node start, Node goal, float maxConnectionDistance) {
	Start = start;
	Goal = goal;

    RapidTree.Add_Vertex(Start, false, 0);
    int k = RapidTree.Get_NumOfVertices() - 1;
    
    // While the Goal has not been reached.
    while (RapidTree.Get_Vertex(k).x == goal.x && 
        RapidTree.Get_Vertex(k).y == goal.y) {

        Node randP = Get_RandomPosition();
        Set_NewVertex(Get_NearestVertexIndex(randP), randP, maxConnectionDistance);
    }
}



void RRT::Run_RRTStar(Node start, Node goal, float maxConnectionDistance, float search_radius) {
	Start = start;
	Goal = goal;

	SearchRadius = search_radius;
	RapidTree.Add_Vertex(Start, false, 0);
    int k = RapidTree.Get_NumOfVertices() - 1;
    
    // While the Goal has not been reached.
    while (RapidTree.Get_Vertex(k).x == goal.x && 
        RapidTree.Get_Vertex(k).y == goal.y) {

        Node randP = Get_RandomPosition();
		std::vector<int> neighbors = Get_Neighbors(SearchRadius, randP);
		std::pair<int, float> nearest_index = Get_NearestVertexIndex(randP);
		std::pair<int, float> best_index_in_radius = Get_BestNeighbor(neighbors, randP);
		
		if (best_index_in_radius.second <= nearest_index.second) {
			Set_NewVertex(best_index_in_radius, randP, maxConnectionDistance);
		}

		else { Set_NewVertex(nearest_index, randP, maxConnectionDistance); }

		// Re-wire other neighbors in search radius
		int randP_index = RapidTree.Get_Graph().back().VertexID;
		Rewire_Neighbors(neighbors, randP_index, randP);
    }
}




/*
 * 			TO-DO
 * 			-----
 *  - In Rewire_Neighbors Function: Recalculate the DistanceFromStart for all children of i.
 *
 *  - Test Code
 *
 *  - 
 *  */
