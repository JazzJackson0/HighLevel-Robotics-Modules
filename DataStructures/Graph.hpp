#include <iostream>
#include <list>
#include <vector>
using std::list; // Linked Lists


template <typename Z>
struct Edge {
    int AdjacentVertex_ID;
    Z Weight;
};



template <typename T, typename Z>
struct Vertex {
    Vertex(int id, std::list<Edge<Z>> &adj) : Vertex_ID(id), Adjacents(adj) {} // Struct Constructor
    int Vertex_ID;
    T Data;
    std::list<Edge<Z>> Adjacents;
};



template <typename T, typename Z>
class Graph {

    private:
        bool Directed = false;
        std::vector<Vertex<T, Z>> G;    
    
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
        Graph(std::vector<Vertex<T, Z>> adjacency_list, bool state);


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
         * @return ** Vertex<T, Z> - The newly added Vertex 
         */
        Vertex<T, Z> Add_Vertex(T data);


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
         * @brief Returns a list of all edges (with their correspoinding vertices) adjacent to the given vertex
         * 
         * @param vertex_ID ID of the vertex whose 
         * @return ** std::list<Edge<Z>> 
         */
        std::list<Edge<Z>> Get_AdjacentVertices(int vertex_ID);


        /**
         * @brief Connect new Vertex to another in the graph.
         * 
         * @param new_vertex_data The data that will make the new Vertex
         * @param vertexConnect_ID ID of the Vertex in the Graph that will connect to 
         *                          the new Vertex.
         * @param weight Weight of the connection between Vertices.
         * @return ** Vertex<T, Z> - The newly added Vertex  
         */
        Vertex<T, Z> Connect_NewVertex(T new_vertex_data, int vertexConnect_ID, Z weight);

        // NEW !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        /**
         * @brief Connect new Vertex to the previously added vertex
         * 
         * @param new_vertex_data The data that will make the new Vertex
         * @param weight Weight of the connection between Vertices.
         * @return ** Vertex<T, Z> - The newly added Vertex  
         */
        Vertex<T, Z> AutoConnect_NewVertex(T new_vertex_data, Z weight);


        /**
         * @brief C
         * 
         * @return ** void 
         */
        Vertex<T, Z> Get_PreviousVertex();

        // NEW !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        


        /**
         * @brief Returns all Vertices in the form of a pair.
         *          ||| First Element in Pair: Number of Vertices in Graph.
         *          ||| Second Element in Pair: Vector array of all Vertices.
         * 
         * @return ** std::vector<Vertex<T, Z>>
         */
        std::vector<Vertex<T, Z>> Get_Vertices();


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