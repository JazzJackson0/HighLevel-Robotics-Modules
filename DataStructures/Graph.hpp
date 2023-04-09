#pragma once
#include <iostream>
#include <list>
#include <vector>
#include <queue>

using std::list; // Linked Lists
using std::vector;



template <typename Z> struct Edge {
    int AdjacentVertexID;
    Z Weight;
    int err;
};




template <typename T, typename Z> struct Vertex {
    //Vertex(int id, vector<Edge<Z>> &adj) : VertexID(id), Adjacents(adj) {} // Struct Constructor
    int VertexID;
    T Data;
    vector<Edge<Z>> Adjacents;
    int adjacents_pointer;
    int err;
};



// Declaration----------------------------------------------------------------------------------------
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
         * @brief Returns the edge between 2 vertices
         * 
         * @param idx1
         * @param idx2
         * @return ** Z 
         */
        Z Get_Edge(int idx1, int idx2);


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
        void Add_Edge(int vertex1_ID, int vertex2_ID, Z weight);


        /**
		 * @brief Removes the edge from between 2 vertices
		 *
		 * @param index1 Index of vertex 1
		 * @param index2 Index of vertex 2
		 *
		 * @return ** void
		 */
        void Remove_Edge(int index1, int index2);


        /**
         * @brief Updates (overwrites) the data in the given vertex with data passed as argument
         * 
         * @param vertex_id vertex to update
         * @param data new data
         */
        void Update_Data(int vertex_id, T data);


        /**
         * @brief Updates (overwrites) the edge data (from id1 to id2 for directed & between both for undirected) 
         * 
         * @param idx1 index of vertex whose edge will be updated
         * @param idx2 index of vertex whose edge will be updated
         * @param data new data
         */
        void Update_Edge(int idx1, int idx2, Z data);


        /**
         * @brief Performs Depth First Search. (Recursively)
         * 
         * @param target_data 
         * @param current_vertex 
         * @param visited - Array of visited vertices in the graph
         * @return Vertex<T, Z> Returns the vertex if found, NULL if not.
         */
        Vertex<T, Z> DFS(T target_data, int current_vertex, vector<int> visited);


        /**
         * @brief Performs Breadth First Search.
         * 
         * @param target_data 
         * @param current_vertex 
         * @return Vertex<T, Z> Returns the vertex if found, NULL if not.
         */
        Vertex<T, Z> BFS(T target_data, int current_vertex);

        
    
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



// Definition----------------------------------------------------------------------------------------
template <typename T, typename Z>        
Graph<T, Z>::Graph() {

    Directed = false;
    RecentVertexID = 0;
}


template <typename T, typename Z>        
Graph<T, Z>::Graph(std::vector<Vertex<T, Z>> adjacency_list, bool state) : G(adjacency_list) {

    Directed = state;
    G = adjacency_list;
    RecentVertexID = 0;
}



template <typename T, typename Z>
bool Graph<T, Z>::isDirected() {

    return Directed;
}



template <typename T, typename Z>
int Graph<T, Z>::Get_NumOfVertices() {

    return G.size();
}

template <typename T, typename Z>
int Graph<T, Z>::Get_NumOfEdges() {

    return NumOfEdges;
}



template <typename T, typename Z>
T Graph<T, Z>::Get_Vertex(int vertex_id) {

    if (vertex_id < 0 || vertex_id >= G.size()) {
        std::cout << "This Vertex Index doesn't exist." << std::endl;
    }
    
    return G[vertex_id].Data;
}


template <typename T, typename Z>
Z Graph<T, Z>::Get_Edge(int idx1, int idx2) {

    if ((idx1 < 1 || idx1 > G.size()) || (idx2 < 1 || idx2 > G.size())) {

        std::cerr << "Invalid Index" << std::endl;
    }

    for (int i = 0; i < G[idx1].Adjacents.size(); i++) {

        if (G[idx1].Adjacents[i].AdjacentVertexID == idx2)
            return G[idx1].Adjacents[i].Weight;
    }
}



template <typename T, typename Z>
std::vector<Vertex<T, Z>> Graph<T, Z>::Get_Graph() {
    
    return G;
}


template <typename T, typename Z>
std::vector<Edge<Z>> Graph<T, Z>::Get_AdjacentVertices(int vertex_id) {

    return G[vertex_id].Adjacents;
}


template <typename T, typename Z>
Vertex<T, Z> Graph<T, Z>::Add_Vertex(T data, bool connected, Z weight) {

    Vertex<T, Z> new_vertex;
    new_vertex.Data = data;
    new_vertex.VertexID = G.size();
    new_vertex.adjacents_pointer = 0;
    new_vertex.err = 0;
    G.push_back(new_vertex);

    if (connected) {

        Add_Edge(RecentVertexID, new_vertex.VertexID, weight);
    }

    RecentVertexID = G.size();
    return new_vertex;
}


template <typename T, typename Z>
void Graph<T, Z>::Add_Edge(int vertex1_ID, int vertex2_ID, Z weight) {

    if ((vertex1_ID < 0 || vertex1_ID > G.size()) || (vertex2_ID < 0 || vertex2_ID > G.size())) {

        std::cerr << "Invalid Index" << std::endl;
        return;
    }

    // Edge from V1 to V2
    Edge<Z> new_edge;
    new_edge.AdjacentVertexID = vertex2_ID;
    new_edge.Weight = weight;
    G[vertex1_ID].Adjacents.push_back(new_edge);

    // Edge from V2 to V1 (If Undirected)
    if (!Directed) {
        Edge<Z> new_edge2;
        new_edge2.AdjacentVertexID = vertex1_ID;
        new_edge2.Weight = weight;
        G[vertex2_ID].Adjacents.push_back(new_edge2);
    }

    NumOfEdges++;
}


template <typename T, typename Z>
void Graph<T, Z>::Remove_Edge(int index1, int index2) {

    if ((index1 < 1 || index1 > G.size()) || (index2 < 1 || index2 > G.size())) {

        std::cerr << "Invalid Index" << std::endl;
        return;
    }

    // Vector iterators are less effiicient than list ones at removals from the middle, but it'll work for now. 
    typename std::vector<Edge<Z>>::iterator iter1 = G[index1].Adjacents.begin();
    typename std::vector<Edge<Z>>::iterator iter2 = G[index2].Adjacents.begin();
    //iter1 = G[index1].Adjacents.begin();
    //iter2 = G[index2].Adjacents.begin();

    // Remove Edge from Index 1 Side
    for (int i = 0; i < G[index1].Adjacents.size(); i++) {

        if (G[index1].Adjacents[i].AdjacentVertexID == index2) {

            G[index1].Adjacents.erase(iter1);
            break;
        }
        else
            iter1++;
    }

    // Remove Edge from Index 2 Side
    for (int i = 0; i < G[index2].Adjacents.size(); i++) {

        //iter2 = G[index2].begin();

        if (G[index2].Adjacents[i].AdjacentVertexID == index1) {

            G[index2].Adjacents.erase(iter2);
            break;
        }
        else
            iter2++;
    }

    NumOfEdges--;

}



template <typename T, typename Z>
void Graph<T, Z>::Update_Data(int vertex_id, T data) {

    if (vertex_id < 0 || vertex_id >= G.size()) {
        std::cout << "This Index doesn't exist." << std::endl;
        return;
    }
    
    G[vertex_id].Data = data;
}



template <typename T, typename Z>
void Graph<T, Z>::Update_Edge(int idx1, int idx2, Z data) {

    if ((idx1 < 1 || idx1 > G.size()) || (idx2 < 1 || idx2 > G.size())) {

        std::cerr << "Invalid Index" << std::endl;
        return;
    }

    // Update idx1 edge
    for (int i = 0; i < G[idx1].Adjacents.size(); i++) {

        if (G[idx1].Adjacents[i].AdjacentVertexID == idx2) {
            G[idx1].Adjacents[i].Weight = data;

            // Update idx2 edge
            if (!Directed) {

                for (int j = 0; j < G[idx2].Adjacents.size(); j++) {
                    if (G[idx2].Adjacents[j].AdjacentVertexID == idx1)
                        G[idx2].Adjacents[j].Weight = data;
                }
            }
        }
    }
}


template <typename T, typename Z>
Vertex<T, Z> Graph<T, Z>::DFS(T target_data, int current_vertex, std::vector<int> visited) {

    Vertex<T, Z> error;

    if (current_vertex < 0 || current_vertex > G.size()){
        std::cerr << "Invalid Vertex ID" << std::endl;
        error.err = 0;
        return error;
    }
    
    if (G[current_vertex].Data == target_data)
        return G[current_vertex];


    // Add vertex to visited list
    visited.push_back(current_vertex);

    // Loop through each adjacent vertex
    bool been_visited = false;
    for (int i = 0; i < G[current_vertex].Adjacents.size(); i++) {

        // Check if the adjacent has been visited.
        for (int j = 0; j < visited.size(); j++) {

            if (G[current_vertex].Adjacents[i].AdjacentVertexID == visited[j])
                been_visited = true;
        }

        // If not visited, continue depth search into adjacent
        if (!been_visited) 
            DFS(target_data, G[current_vertex].Adjacents[i].AdjacentVertexID, visited);

        else
            been_visited = false;
    }

    // Can't go deeper
    if (visited.size() == G.size())
        error.err = 1;
        return error;     
}


template <typename T, typename Z>
Vertex<T, Z> Graph<T, Z>::BFS(T target_data, int current_vertex) {

    Vertex<T, Z> error;

    if (current_vertex < 0 || current_vertex > G.size()){
        std::cerr << "Invalid Vertex ID" << std::endl;
        error.err = 0;
        return error;
    }
    
    if (G[current_vertex].Data == target_data)
        return G[current_vertex];

    vector<int> visited;
    std::queue<int> task_queue;
    // Enqueue first vertex to task queue and mark as visited
    visited.push_back(current_vertex);
    task_queue.push(current_vertex);

    // Perform search task on each vertex in queue
    while (task_queue.size() > 0) {

        // Dequeue vertex
        int vertex = task_queue.front();
        task_queue.pop();

        // Loop through dequeued vertex's adjacent vertices
        bool been_visited = false;
        for (int i = 0; i < G[current_vertex].Adjacents.size(); i++) {

            // Check if the adjacent has been visited.
            for (int j = 0; j < visited.size(); j++) {

                if (G[current_vertex].Adjacents[i].AdjacentVertexID == visited[j])
                    been_visited = true;
            }

            // If not visited, mark it as visited and enqueue it
            if (!been_visited) {    
                int adjacent_id = G[current_vertex].Adjacents[i].AdjacentVertexID;
                
                // Return the adjacent vertex if data found
                if (G[adjacent_id].Data == target_data)
                    return G[adjacent_id];

                // Enqueue adjacent vertex to search task queue & mark as visited.
                else {
                    visited.push_back(adjacent_id);
                    task_queue.push(adjacent_id);
                }
            }

            else
                been_visited = false;
        }
    }

    error.err = 1;
    return error;
}



template <typename T, typename Z>
void Graph<T, Z>::Print_Vertices() {

    for (int i = 0; i < G.size(); i++) {

        std::cout << G[i].VertexID << " -> " << G[i].Data << std::endl;
    }
}



template <typename T, typename Z>
void Graph<T, Z>::Print_Edges() {
    
    for (int i = 0; i < G.size(); i++) {

        for (int j = 0; j < G[i].Adjacents.size(); j++) {

            std::cout << G[i].VertexID << "-" << G[i].Adjacents[j].AdjacentVertexID << " ";
        }
        
        std::cout << "\n";
    }
}



/*
 * 			TO-DO
 * 			-----
 *  - Test Code
 *
 *  - 
 *  
 *  - 
 *  */