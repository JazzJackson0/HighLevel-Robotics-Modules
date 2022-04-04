#pragma once
#include <iostream>
#include <vector>
using std::vector;
using std::pair;
using std::make_pair;

typedef struct edge Edge;

//--------------------------------------------------------------------------
template <typename T, typename Z> class Vertex {
    private:
        T Data;
        vector<Edge> Neighbors;
        int NumOfNeighbors;
        int VertexSet_Index;
    public:
        
        Vertex();

        /**
         * @brief Create a Vertex to hold given data.
         * 
         * @param data Data to be stored in the vertex. 
         * 
         */
        Vertex(T data);

        /**
         * @brief Add data to a given Vertex.
         * 
         * @param data Data to place in Vertex
         * @return ** void 
         */
        void Add_Data(T data);

        /**
         * @brief Returns the data stored in a given Vertex.
         * 
         * @return ** T - Data to be returned.
         */
        T Get_Data();

        /**
         * @brief Connects to a new Vertex with a weighted Edge (struct).
         *          The new Vertex is added to this Vertex's list of Neighbors.
         * 
         * @param newVertex The new Vertex this one will connect to.
         * @param weight The weight asigned to the Edge.
         * @return ** void 
         */
        void Connect_Vertex(Vertex<T, Z> newVertex, Z weight);

        /**
         * @brief Print all edges connected to a given vertex.
         * 
         * @return ** void 
         */
        void Print_Edges();

        /**
         * @brief Set an ID number for this Vertex.
         *          Allows a Graph to better keep track of its Vertices
         *          (e.g. indexing).
         * 
         * @param ID Vertex ID 
         * @return ** void 
         */
        void Set_VertexID(int ID);

        /**
         * @brief Returns the ID number of this Vertex
         * 
         * @return ** int 
         */
        int Get_VertexID();

		/**
		 * @brief Removes the edge that connecte to the vertex corresponding to the passsed ID
		 *
		 * @param vertexID ID number of the vertex to be disconnected from this one.
		 * 
		 * @return ** void
		 */
		void Remove_Edge(int vertexID);
};

//template <typename T, typename Z>
struct edge {
    template <typename T, typename Z>
	Vertex<T, Z> NeighborVertex; 
    Z Edge_Weight; 
};
//----------------------------------------------------------------------------


template <typename T, typename Z>
using VertexSet = pair<int, vector<Vertex<T, Z>>>;
// VertexSet = <n Vertices in Set, The Set of Vertices>
//----------------------------------------------------------------------------

template <typename T, typename Z> class Graph {
    private:
        bool directed = false;
        VertexSet VSet; 
    public:

        /**
         * @brief 
         * 
         */
        Graph();

        /**
         * @brief Create a Graph
         * 
         * @param state Sets Graph to be Directed (TRUE) or Undirected (FALSE) 
         * 
         */
        Graph(bool state);

        /**
         * @brief Checks it Graph is Directed or Undirected
         * 
         * @return true 
         * @return false 
         */
        bool isDirected();

        /**
         * @brief Returns the number of Vertices in the Vertex set
         * 
         * @return ** int - Number of Vertices in Vertex Set
         */
        int Get_NumOfVertices();

        /**
         * @brief Returns the data of the Vertex in the Vertex Set which 
         *          corresponds to the given ID.
         * 
         * @param vertex_ID ID of the Vertex whose data will be returned
         * @return ** T Data of the desired Vertex
         */
        T Get_Vertex(int vertex_ID);
        
        /**
         * @brief Adds a vertex to the Graph.
         * 
         * @param data Data to be added.
         * @return ** void 
         */
        void Add_Vertex(T data);

        /**
         * @brief Makes an edge connection between 2 Vertices in the Graph.
         * 
         * @param vertex1_ID ID of Vertex 1 in the connection.
         * @param vertex2_ID ID of Vertex 2 in the connection.
         * @param weight Weight of the edge between the Vertices.
         * @return ** void 
         */
        void Make_Connection(int vertex1_ID, int vertex2_ID, Z weight);

        /**
         * @brief Connect new Vertex to another in the graph.
         * 
         * @param vertexConnect_ID ID of the Vertex in the Graph that will connect to 
         *                          the new Vertex.
         * @param new_vertex_data The data that will make the new Vertex
         * @param weight Weight of the connection between Vertices.
         * @return ** void 
         */
        void Connect_NewVertex(int vertexConnect_ID, T new_vertex_data, Z weight);

        /**
         * @brief Returns all Vertices in the form of a pair.
         *          ||| First Element in Pair: Number of Vertices in Graph.
         *          ||| Second Element in Pair: Vector array of all Vertices.
         * 
         * @return ** pair<int, vector<Vertex<T>>>
         */
        pair<int, vector<Vertex<T, Z>>> Get_Vertices();

        /**
         * @brief Prints all Vertices in the Graph.
         * 
         * @return ** void 
         */
        void Print_Vertices();


        /**
         * @brief Prints all edges in the Graph.
         * 
         * @return ** void 
         */
        void Print_Edges();


		/**
		 * @brief Removes the edge from between 2 vertices
		 *
		 * @param index1 Index of vertex 1
		 * @param index2 Index of vertex 2
		 *
		 * @return ** void
		 */
		void Disconnect_Vertices(int index1, int index2) {


        /**
         * @brief Performs Depth First Search. (Recursively)
         * 
         * @param starting_vertex 
         * @param target_vertex 
         * @param Visited 
         * @return ** void 
         */
        void DFS(Vertex<T, Z> starting_vertex, Vertex<T, Z> target_vertex, 
            vector<Vertex<T, Z>> Visited);

        /**
         * @brief Performs Breadth First Search.
         * 
         * @param starting_vertex 
         * @param target_vertex 
         * @return ** void 
         */
        void BFS(Vertex<T, Z> starting_vertex, Vertex<T, Z> target_vertex);
};




