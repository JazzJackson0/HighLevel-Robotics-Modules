#pragma once
#include <iostream>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include "../DataStructures/Graph.hpp"
using std::make_pair;

struct node {	
	int x;
	int y;
	int DistanceFromStart;
	int ParentIndex;
};
typedef struct node Node;

class RRT {

    private:
        int G_HEIGHT;
        int G_WIDTH;
        int** Grid;
        Graph<Node, int> RapidTree;
		float SearchRadius;
		Node Start;
		Node Goal;

        /**
         * @brief Find the Vertex in the Tree that is nearest to the given Random (x,y)
         *          coordinates.
         * 
         * @param randPos Random Position
		 *
         * @return ** pair<int, float> - (Index of the Nearest Vertex, Distance to Random Position)
         */
        std::pair<int, float> Get_NearestVertexIndex(Node randPos);

        /**
         * @brief Produce a random (x, y) coordinate pair within the bounds of the grid.
         * 
         * @return ** Node - A Random (x, y) position on the grid.
         */
        Node Get_RandomPosition();

        /**
         * @brief Set new Vertex at the Max Connection Distance in the direction of the 
         *          Random Coordinate. 
         * 
         * @param nearestVIndex A pair: (Vertex Index within Graph, Distance from Random Position).
         * @param randPos Random Position (x, y) coordinates
         * @param maxConnectDist Maximum distance that a Vertex can be from another Vertex 
         *                          it's connected to.
         * @return true - If new Vertex was set.
         * @return false - If new Vertex was not set.
         */
        bool Set_NewVertex(std::pair<int, float> nearestVIndex, Node randPos, 
            int maxConnectDist);
		
		/**
		 * @brief Produce a vector of all nodes within a given radius of the random node.
		 *
		 * @param search_radius The radius around the given node in which to search for nodes.
		 * @param randPos The random node at the center of the search radius.
		 *
		 * @return ** vector<int> - List of neighboring vector indices
		 */
		std::vector<int> Get_Neighbors(float search_radius, Node randPos); 

		/**
		 * @brief Searches through the list of vertices for the vertex with the best cost
         *          (i.e. the smallest distance back to the start position).
		 *
		 * @param neighbors List of vertices within the search radius of a random node.
         * @param randPos The random node at the center of the search radius.
		 *
		 * @return ** pair<int, float> - (Index of neighbor with best Cost, Distance between that node 
         *                                  and the random position)
		 */
    	std::pair<int, float> Get_BestNeighbor(std::vector<int> neighbors, Node randPos);

		/**
		 * @brief Reconnects a given node to whichever one produces the shortest path 
         *          back to the start.
		 *
		 * @param neighbors List of vertices within the search radius of a random node.
		 * @param nodeIndex The node to rewire
         * @param randPos The random node at the center of the search radius.
		 *
		 * @return ** void 
		 */
		void Rewire_Neighbors(std::vector<int> neighbors, int nodeIndex, Node randPos);

		/**
		 * @brief Calculate the Euclidean Distancee between two Nodes
		 *
		 * @param node_a Node A
		 * @param node_b Node B
		 *
		 * @return ** float - The distance between 2 nodes.
		 * **/
		float Get_Distance(Node node_a, Node node_b); 

    public:

        /**
         * @brief Performs either an RRT or an RRT* search on a given Grid.
         * 
         * @param grid The Grid that will be searched
         * @param width Width of the Grid
         * @param height Heigh of the Grid
         * 
         */
        RRT(int **grid, int width, int height);

        /**
         * @brief Runs the RRT algorithm
         * 
         * @param start Start Coordinates
         * @param goal Goal Coordinates
         * @param maxConnectionDistance Maximum distance that a Vertex can be from another Vertex 
         *                          it's connected to.
         * @return ** void 
         */
        void Run_RRT(Node start, Node goal, float maxConnectionDistance);

		/**
		 * @brief Runs an optimized version of RRT that provides a shorter path to the goal than
         *          RRT.
		 *
		 * @param start Start Coordinates
		 * @param goal Goal Coordinates
		 * @param maxConnectionDistance The max distance nodes are allowed to be from each other.
		 * @param search_radius The search radius distance.
		 *
		 * @return ** void 
		 */
        void Run_RRTStar(Node start, Node goal, float maxConnectionDistance, float search_radius);

};





