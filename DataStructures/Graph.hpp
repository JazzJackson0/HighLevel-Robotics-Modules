#include <iostream>
#include <list>
#include <vector>
using std::list; // Linked Lists
using std::vector;


template <typename Z>
struct Edge {
    int AdjacentVertexID;
    Z Weight;
};



template <typename T, typename Z>
struct Vertex {
    Vertex(int id, vector<Edge<Z>> &adj) : VertexID(id), Adjacents(adj) {} // Struct Constructor
    int VertexID;
    T Data;
    vector<Edge<Z>> Adjacents;
    int adjacents_pointer;
};



template <typename T, typename Z>
class Graph {

    private:
        bool Directed;
        vector<Vertex<T, Z>> G;
        int RecentVertexID;  
        int NumOfEdges;  
    
    public:
        

        /**
         * @brief Initialize a Graph
         * 
         */
        Graph();
        
        /**
         * @brief Initialize a Graph
         * 
         * @param adjacency_list The data structure for the Graph. An array of vertices, each storing their own
         *          Linked List of adjacent vertices.
         * @param state Sets Graph to be Directed (TRUE) or Undirected (FALSE) 
         * 
         */
        Graph(vector<Vertex<T, Z>> adjacency_list, bool state);


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
         * @brief Returns the number of Edges in the Vertex set
         * 
         * @return ** int - Number of Edges in Vertex Set
         */
        int Get_NumOfEdges();


        /**
         * @brief Returns the data of the Vertex in the Vertex Set which 
         *          corresponds to the given ID.
         * 
         * @param vertex_id ID of the Vertex whose data will be returned
         * @return ** T Data of the desired Vertex
         */
        T Get_Vertex(int vertex_id);


        /**
         * @brief Returns all Vertices in the form of a pair.
         *          ||| First Element in Pair: Number of Vertices in Graph.
         *          ||| Second Element in Pair: Vector array of all Vertices.
         * 
         * @return ** vector<Vertex<T, Z>>
         */
        vector<Vertex<T, Z>> Get_Graph();


        /**
         * @brief Returns a list of all edges (with their correspoinding vertices) adjacent to the given vertex
         * 
         * @param vertex_id ID of the vertex whose 
         * @return ** vector<Edge<Z>> 
         */
        vector<Edge<Z>> Get_AdjacentVertices(int vertex_id);


        /**
         * @brief Adds a vertex to the Graph. If 'connected' is True the vertex will connect to the most recently
         *      added vertex. Else, the vertex remains in the graph unconnected.
         * 
         * @param data Data to be added.
         * @param connected IF false, leave vertex unconnected, If true, connect vertex to most recent vertex
         * @param weight Applied to the edge between the connected vertices if 'connected' is True
         * @return ** Vertex<T, Z> - The newly added Vertex 
         */
        Vertex<T, Z> Add_Vertex(T data, bool connected, Z weight);


        /**
         * @brief Makes an edge connection between 2 Vertices in the Graph.
         * 
         * @param vertex1_ID ID of Vertex 1 in the connection.
         * @param vertex2_ID ID of Vertex 2 in the connection.
         * @param weight Weight of the edge between the Vertices.
         * @return ** void 
         */
        void Connect_Vertices(int vertex1_ID, int vertex2_ID, Z weight);


        /**
		 * @brief Removes the edge from between 2 vertices
		 *
		 * @param index1 Index of vertex 1
		 * @param index2 Index of vertex 2
		 *
		 * @return ** void
		 */
        void Disconnect_Vertices(int index1, int index2);


        /**
         * @brief Performs Depth First Search. (Recursively)
         * 
         * @param target_data 
         * @return ** Returns the vertex if found, NULL if not.
         */
        Vertex<T, Z> DFS(T target_data);


        /**
         * @brief Performs Breadth First Search.
         * 
         * @param starting_vertex 
         * @param target_vertex 
         * @return ** Returns the vertex if found, NULL if not.
         */
        Vertex<T, Z> BFS(T target_data);

        
    
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
              
};



/*
 * 			TO-DO
 * 			-----
 *  - Test Code
 *
 *  - 
 *  
 *  - 
 *  */