#pragma once
#include <iostream>
#include <list>
#include <vector>
#include <queue>

using std::list; // Linked Lists
using std::vector;



template <typename Z> struct Edge {
    int edge_id;
    int vertexA;
    int vertexB;
    Z weight;
    int err;
};




template <typename T, typename Z> struct Vertex {
    //Vertex(int id, vector<Edge<Z>> &adj) : id(id), Adjacents(adj) {} // Struct Constructor
    int id;
    T data;
    vector<int> edges;
    vector<int> adjacents;
    int err;
};



// Declaration----------------------------------------------------------------------------------------
template <typename T, typename Z>
class Graph {

    private:
        bool Directed;
        std::vector<Vertex<T, Z>> G;
        std::vector<Edge<Z>> Edges;
        int RecentVertexID;   
    
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
         * @brief Returns the number of Vertices in the Graph
         * 
         * @return ** int - Number of Vertices in Graph
         */
        int Get_NumOfVertices();


        /**
         * @brief Returns the number of Edges in the Graph
         * 
         * @return ** int - Number of Edges in Graph
         */
        int Get_NumOfEdges();


        /**
         * @brief Returns the data of the Vertex in the Graph which 
         *          corresponds to the given ID.
         * 
         * @param vertex_id ID of the Vertex whose data will be returned
         * @return ** T Data of the desired Vertex
         */
        T Get_Vertex(int vertex_id);


        /**
         * @brief Returns the degree of the vertex
         * 
         * @param vertex_id ID of the Vertex
         * @return int Degree
         */
        int Get_Degree(int vertex_id);
        

        /**
         * @brief Returns one of the vertexes incident edges as specified by the edge_idx
         * 
         * @param vertex_id 
         * @param edge_idx index of edge in vertex's edges vector
         * @return Z Edge
         */
        Z Get_Edge(int vertex_id, int edge_idx);


        /**
         * @brief Returns the indices of the two vertex ends corresponding to the given edge
         *  
         * @param edge_idx index of edge in global Edge vector.
         * @return pair<int, int>
         */
        std::pair<int, int> Get_EdgeEnds(int edge_idx);


        /**
         * @brief Returns one of the vertices adjacent (as specified by the adjacent_id) to 'vertex_id' 
         * 
         * @param vertex_id 
         * @param adjacent_id 
         * @return T 
         */
        T Get_AdjacentVertex(int vertex_id, int adjacent_id);

        /**
         * @brief Returns the edge between 2 vertices
         * 
         * @param idx1
         * @param idx2
         * @return ** Z Edge
         */
        Z Get_AdjacentEdge(int idx1, int idx2);


        /**
         * @brief Returns all Vertices in the form of a pair.
         *          ||| First Element in Pair: Number of Vertices in Graph.
         *          ||| Second Element in Pair: Vector array of all Vertices.
         * 
         * @return ** vector<Vertex<T, Z>>
         */
        vector<Vertex<T, Z>> Get_Graph();


        /**
         * @brief Adds a vertex to the Graph. If 'connected' is True the vertex will connect to the most recently
         *      added vertex. Else, the vertex remains in the graph unconnected.
         * 
         * @param data Data to be added.
         * @param connected IF false, leave vertex unconnected, If true, connect vertex to most recent vertex
         * @param weight Applied to the edge between the connected vertices if 'connected' is True
         * @return ** T - Data of the newly added Vertex 
         */
        T Add_Vertex(T data, bool connected, Z weight);


        /**
         * @brief Makes an edge connection between 2 Vertices in the Graph.
         * 
         * @param vertex1_ID ID of Vertex 1 in the connection.
         * @param vertex2_ID ID of Vertex 2 in the connection.
         * @param weight weight of the edge between the Vertices.
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

    return Edges.size();
}



template <typename T, typename Z>
T Graph<T, Z>::Get_Vertex(int vertex_id) {

    if (vertex_id < 0 || vertex_id >= G.size()) {
        std::cout << "Error: This Vertex Index doesn't exist." << std::endl;
    }
    
    return G[vertex_id].data;
}

template <typename T, typename Z>
int Graph<T, Z>::Get_Degree(int vertex_id) {

    return G[vertex_id].adjacents.size();
}

template <typename T, typename Z>
Z Graph<T, Z>::Get_Edge(int vertex_id, int edge_idx) {

    if (edge_idx < 0 || edge_idx >= G[vertex_id].edges.size()) {
        std::cerr << "Error: Edge Index " << edge_idx << " is Invalid. Cannot get Edge." << std::endl;
    }

    if (vertex_id < 0 || vertex_id >= G.size()) {
        std::cerr << "Error: Vertex Index " << vertex_id << " is Invalid. Cannot get Edge." << std::endl;
    }

    int index = G[vertex_id].edges[edge_idx];
    return Edges[index].weight;
}

template <typename T, typename Z>
std::pair<int, int> Graph<T, Z>::Get_EdgeEnds(int edge_idx) {

    if (edge_idx < 0 || edge_idx >= Edges.size()) {
        std::cerr << "Error: Edge Index " << edge_idx << " is Invalid. Cannot get Edge." << std::endl;
    }

    return std::make_pair(Edges[edge_idx].vertexA, Edges[edge_idx].vertexB);

}

template <typename T, typename Z>
T Graph<T, Z>::Get_AdjacentVertex(int vertex_id, int adjacent_id) {

    if (adjacent_id < 0 || adjacent_id >= G[vertex_id].adjacents.size()) {
        std::cerr << "Error: Adjacent Index " << adjacent_id << " is Invalid. Cannot get Vertex." << std::endl;
    }

    if (vertex_id < 0 || vertex_id >= G.size()) {
        std::cerr << "Error: Vertex Index " << vertex_id << " is Invalid. Cannot get Vertex." << std::endl;
    }

    int index = G[vertex_id].adjacents[adjacent_id];
    return G[index].data;
}


template <typename T, typename Z>
Z Graph<T, Z>::Get_AdjacentEdge(int idx1, int idx2) {

    if ((idx1 < 0 || idx1 >= G.size())) {

        std::cerr << "Error: Index " << idx1 << " is Invalid. Cannot get Edge." << std::endl;
    }

    if ((idx2 < 0 || idx2 >= G.size())) {

        std::cerr << "Error: Index " << idx2 << " is Invalid. Cannot get Edge." << std::endl;
    }

    if (idx1 == idx2) {

        std::cerr << "Error: Index " << idx1 << " = Index " << idx2 << std::endl;
    }

    for (int i = 0; i < G[idx1].adjacents.size(); i++) {

        int edge_idx = G[idx1].adjacents[i];

        if (Edges[edge_idx].vertexA == idx2 || Edges[edge_idx].vertexB == idx2)
            return Edges[edge_idx].weight;
    }
}



template <typename T, typename Z>
std::vector<Vertex<T, Z>> Graph<T, Z>::Get_Graph() {
    
    return G;
}


template <typename T, typename Z>
T Graph<T, Z>::Add_Vertex(T data, bool connected, Z weight) {

    Vertex<T, Z> new_vertex;
    new_vertex.data = data;
    new_vertex.id = G.size();
    new_vertex.err = 0;
    G.push_back(new_vertex);

    if (connected) {

        Add_Edge(RecentVertexID, new_vertex.id, weight);
    }

    RecentVertexID = new_vertex.id;
    return new_vertex.data;
}


template <typename T, typename Z>
void Graph<T, Z>::Add_Edge(int vertex1_ID, int vertex2_ID, Z weight) {

    if ((vertex1_ID < 0 || vertex1_ID > G.size() - 1)) {

        std::cerr << "Error: Index " << vertex1_ID << " is Invalid. Cannot remove Edge." << std::endl;
        return;
    }

    if ((vertex2_ID < 0 || vertex2_ID > G.size() - 1)) {

        std::cerr << "Error: Index " << vertex2_ID << " is Invalid. Cannot remove Edge." << std::endl;
        return;
    }

    
    Edge<Z> new_edge;
    new_edge.edge_id = Edges.size();
    new_edge.vertexA = vertex1_ID;
    new_edge.vertexB = vertex2_ID;
    new_edge.weight = weight;

    // Edge from V1 to V2
    G[vertex1_ID].edges.push_back(new_edge.edge_id);
    G[vertex1_ID].adjacents.push_back(vertex2_ID);

    // Edge from V2 to V1 (If Undirected)
    if (!Directed) {
        G[vertex2_ID].edges.push_back(new_edge.edge_id);
        G[vertex2_ID].adjacents.push_back(vertex1_ID);
    }

    Edges.push_back(new_edge);
}


// TODO: Not Finished
template <typename T, typename Z>
void Graph<T, Z>::Remove_Edge(int index1, int index2) {

    if ((index1 < 0 || index1 > G.size() - 1)) {

        std::cerr << "Error: Index " << index1 << " is Invalid. Cannot remove Edge." << std::endl;
        return;
    }

    if ((index2 < 0 || index2 > G.size() - 1)) {

        std::cerr << "Error: Index " << index2 << " is Invalid. Cannot remove Edge." << std::endl;
        return;
    }

    // Vector iterators are less effiicient than list ones at removals from the middle, but it'll work for now. 
    typename std::vector<int>::iterator iter1 = G[index1].edges.begin();
    typename std::vector<int>::iterator iter2 = G[index2].edges.begin();

    // Remove Edge from Index 1 Side
    for (int i = 0; i < G[index1].adjacents.size(); i++) {

        if (G[index1].adjacents[i] == index2) {

            // TODO: Also remove attached vertex from the 'adjacents' array
            G[index1].edges.erase(iter1);
            break;
        }
        else
            iter1++;
    }

    // Remove Edge from Index 2 Side
    for (int i = 0; i < G[index2].adjacents.size(); i++) {

        if (G[index2].adjacents[i] == index1) {

            // TODO: Also remove attached vertex from the 'adjacents' array
            G[index2].edges.erase(iter2);
            break;
        }
        else
            iter2++;
    }

    // TODO: Currently no way to handle updating the number of edges. 
        // If you delete edge from somewhere in middle, the edge index order is thrown off

}



template <typename T, typename Z>
void Graph<T, Z>::Update_Data(int vertex_id, T data) {

    if (vertex_id < 0 || vertex_id >= G.size()) {
        std::cout << "This Index doesn't exist." << std::endl;
        return;
    }
    
    G[vertex_id].data = data;
}



template <typename T, typename Z>
void Graph<T, Z>::Update_Edge(int idx1, int idx2, Z data) {

    if ((idx1 < 0 || idx1 >= G.size())) {

        std::cerr << "Error: Index " << idx1 << " is Invalid. Cannot update Edge." << std::endl;
    }

    if ((idx2 < 0 || idx2 >= G.size())) {

        std::cerr << "Error: Index " << idx2 << " is Invalid. Cannot update Edge." << std::endl;
    }

    // Update edge
    for (int i = 0; i < G[idx1].adjacents.size(); i++) {
        int edge_idx1 = G[idx1].edges[i];
        if (G[idx1].adjacents[i] == idx2) {
            Edges[edge_idx1].weight = data;
            break;
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
    
    if (G[current_vertex].data == target_data)
        return G[current_vertex];


    // Add vertex to visited list
    visited.push_back(current_vertex);

    // Loop through each adjacent vertex
    bool been_visited = false;
    for (int i = 0; i < G[current_vertex].adjacents.size(); i++) {

        // Check if the adjacent has been visited.
        for (int j = 0; j < visited.size(); j++) {

            if (G[current_vertex].adjacents[i] == visited[j])
                been_visited = true;
        }

        // If not visited, continue depth search into adjacent
        if (!been_visited) 
            DFS(target_data, G[current_vertex].adjacents[i], visited);

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
    
    if (G[current_vertex].data == target_data)
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
        for (int i = 0; i < G[current_vertex].adjacents.size(); i++) {

            // Check if the adjacent has been visited.
            for (int j = 0; j < visited.size(); j++) {

                if (G[current_vertex].adjacents[i] == visited[j])
                    been_visited = true;
            }

            // If not visited, mark it as visited and enqueue it
            if (!been_visited) {    
                int adjacent_id = G[current_vertex].adjacents[i];
                
                // Return the adjacent vertex if data found
                if (G[adjacent_id].data == target_data)
                    return G[adjacent_id];

                // Enqueue adjacent vertex to search task queue & mark as visited.
                else {
                    visited.push_back(adjacent_id);
                    task_queue.push(adjacent_id);
                }
            }

            else
                been_visited = -1;
        }
    }

    error.err = 1;
    return error;
}



template <typename T, typename Z>
void Graph<T, Z>::Print_Vertices() {

    for (int i = 0; i < G.size(); i++) {

        std::cout << G[i].id << " -> " << G[i].data << std::endl;
    }
}



template <typename T, typename Z>
void Graph<T, Z>::Print_Edges() {
    
    for (int i = 0; i < G.size(); i++) {

        for (int j = 0; j < G[i].edges.size(); j++) {

            std::cout << G[i].id << "- " << G[i].edges[j].vertexA << "<->" << G[i].edges[j].vertexB << "  ";
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