#pragma once
#include <iostream>
#include <cmath>
#include <random>
#include <limits>
#include <utility>
#include <vector>
#include <list>
#include <stack>
#include </usr/include/eigen3/Eigen/Dense>
#include "/usr/include/eigen3/unsupported/Eigen/CXX11/Tensor"
#include "../DataStructures/Graph.hpp"
using std::make_pair;
using namespace Eigen;

struct RRT_Node {	
	int x;
	int y;
	int dist_from_start;
	int parent_idx;
    RRT_Node() {}
    RRT_Node(int _x, int _y) : x(_x), y(_y) {}
    bool operator == (RRT_Node otherNode) {
        return (this->x == otherNode.x && this->y == otherNode.y);
    }
};


struct RRT_Edge {

    float distance;
    int parent_index;
    int child_index;
    RRT_Edge() {}
    RRT_Edge(float _distance, int _parent_index, int _child_index) 
        : distance(_distance), parent_index(_parent_index), child_index(_child_index) {}
};

class RRT {

    private:
        int HEIGHT;
        int WIDTH;
        Eigen::Tensor<float, 2> MAP;
        Graph<RRT_Node, RRT_Edge> RapidTree;
		float SearchRadius;
		RRT_Node Start;
		RRT_Node Goal;
        std::random_device rd;
        bool repeat_node;
        int MaxConnectionDistance; // Maximum distance that a Vertex can be from another Vertex it's connected to.


        /**
         * @brief 
         * 
         * @param x 
         * @param y 
         * @return true 
         * @return false 
         */
        bool isValid(int x, int y);


        /**
         * @brief 
         * 
         * @param x 
         * @param y 
         * @return true 
         * @return false 
         */
        bool isBlocked(int x, int y);


        /**
         * @brief 
         * 
         * @param x 
         * @param y 
         * @return true 
         * @return false 
         */
        bool isGoalReached(int x, int y);


        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool isStartAndGoalValid();


        /**
		 * @brief Calculate the Euclidean Distancee between two Nodes
		 *
		 * @param node_a Node A
		 * @param node_b Node B
		 *
		 * @return ** float - The distance between 2 nodes.
		 * **/
		float Get_Distance(RRT_Node node_a, RRT_Node node_b); 


        /**
         * @brief Produce a random (x, y) coordinate pair within the bounds of the grid.
         * 
         * @return ** Node - A Random (x, y) position on the grid.
         */
        RRT_Node Get_RandomPosition();


        /**
         * @brief Produce a vector of all nodes within a given radius of the random node.
		 *
		 * @param search_radius The radius around the given node in which to search for nodes.
		 * @param randPos The random node at the center of the search radius.
         * @param neighbors
         * @return true 
         * @return false 
         */
		bool Get_Neighbors(float search_radius, RRT_Node randPos, std::vector<int> &neighbors); 

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
    	std::pair<int, float> Get_BestNeighbor(std::vector<int> neighbors, RRT_Node randPos);


        /**
         * @brief Find the Vertex in the Tree that is nearest to the given Random (x,y)
         *          coordinates.
         * 
         * @param randPos Random Position
		 *
         * @return ** pair<int, float> - (Index of the Nearest Vertex, Distance to Random Position)
         */
        std::pair<int, float> Get_NearestVertexIndex(RRT_Node randPos);


        /**
         * @brief 
         * 
         * @param node 
         * @param nearest 
         * @return true 
         * @return false 
         */
        bool lineIntersectsWithObstacle(RRT_Node node, RRT_Node nearest);


        /**
         * @brief 
         * 
         * @param node 
         * @param nearest
         * @return bool 
         */
        bool Move_NodeCloser(RRT_Node &node, RRT_Node nearest);

        
        /**
         * @brief Set new Vertex at the Max Connection Distance in the direction of the 
         *          Random Coordinate. 
         * 
         * @param nearest_info A pair: (Vertex Index within Graph, Distance from Random Position).
         * @param new_vertex_data Data to add to to new vertex
         * @return true - If new Vertex was set.
         * @return false - If new Vertex was not set.
         */
        bool Connect_NewVertex(std::pair<int, float> nearest_info, RRT_Node new_vertex_data);
		
		
		/**
		 * @brief Reconnects a given node to whichever one produces the shortest path 
         *          back to the start.
		 *
		 * @param neighbors List of vertices within the search radius of a random node.
		 * @param newest_node_idx The node to rewire
		 *
		 * @return ** void 
		 */
		void Rewire_Neighbors(std::vector<int> neighbors, int newest_node_idx);


        void Update_Distances(int node_index);


        std::stack<VectorXi> PathTraceHelper(int goal_node_idx, int current_node_idx, std::stack<VectorXi> path);

        std::vector<VectorXi> PathTrace(int goal_node_idx, int current_node_idx);


    public:

        /**
         * @brief Default Constructor
         * 
         */
        RRT();

        /**
         * @brief Performs either an RRT or an RRT* search on a given Grid.
         * 
         * @param map The Grid that will be searched
         * 
         */
        RRT(Eigen::Tensor<float, 2> map);


        void Load_MAP(Eigen::Tensor<float, 2> map);


        /**
         * @brief Runs the RRT algorithm
         * 
         * @param start Start Coordinates
         * @param goal Goal Coordinates
         * @param maxConnectionDistance Maximum distance that a Vertex can be from another Vertex 
         *                          it's connected to.
         * @return ** std::vector<VectorXf> - The waypoints of the path  
         */
        std::vector<VectorXi> RRT_Path(VectorXi start, VectorXi goal, float maxConnectionDistance);

		/**
		 * @brief Runs an optimized version of RRT that provides a shorter path to the goal than
         *          RRT.
		 *
		 * @param start Start Coordinates
		 * @param goal Goal Coordinates
		 * @param maxConnectionDistance The max distance nodes are allowed to be from each other.
		 * @param search_radius The search radius distance.
		 *
		 * @return ** std::vector<VectorXf> - The waypoints of the path 
		 */
        std::vector<VectorXi> RRTStar_Path(VectorXi start, VectorXi goal, float maxConnectionDistance, float search_radius);
};





