#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <stack>

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
        int Directed;
        
        std::vector<Vertex<T, Z>> Vertices;
        std::vector<Edge<Z>> Edges;
        int RecentVertexID;   
        typedef enum { NONE, FORWARD, BACKWARD }EdgeDirection;

        /**
         * @brief Helper for DFS_Path()
         * 
         * @param vertex 
         * @param visited 
         * @return true 
         * @return false 
         */
        bool beenVisited(int vertex, std::vector<int> visited);

        /**
         * @brief 
         * 
         */
        void Delete_Edge(int id);

        /**
         * @brief 
         * 
         */
        void Update_EdgeIDs();

        /**
         * @brief 
         * 
         * @param old_edge_id 
         * @param new_edge_id 
         */
        void Update_EdgeIDHelper(int old_edge_id, int vertex_id, int new_edge_id);
    
    public:
        

        /**
         * @brief Initialize a Graph
         * 
         */
        Graph();
        
        /**
         * @brief Initialize a Graph
         * 
         * @param state Sets Graph to be Undirected (0) or Forward Directed (1) Backward Directed (2)
         * 
         */
        Graph(int state);

        /**
         * @brief 
         * 
         * @param state 
         */
        void SetGraphType(int state);


        /**
         * @brief Checks it Graph is Directed or Undirected
         * 
         * @return true 
         * @return false 
         */
        bool isDirected();


        /**
         * @brief Returns the degree of the vertex
         * 
         * @param vertex_id ID of the Vertex
         * @return int Degree
         */
        int Get_Degree(int vertex_id);


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
         * @brief Returns one of the vertexes incident edges as specified by the edge_idx
         * 
         * @param vertex_id 
         * @param edge_idx index of edge in vertex's edges vector
         * @return Z Edge
         */
        Z Get_Edge(int vertex_id, int edge_idx);


        /**
         * @brief 
         * 
         * @param index 
         * @return Z 
         */
        Z Get_EdgeByIndex(int index);


        /**
         * @brief Returns the edge between 2 vertices
         * 
         * @param idx1
         * @param idx2
         * @return ** Z Edge
         */
        Z Get_EdgeBetweenVertices(int idx1, int idx2);


        /**
         * @brief Returns the indices of the two vertex ends corresponding to the given edge
         *  
         * @param edge_idx index of edge in global Edge vector.
         * @return pair<int, int>
         */
        std::pair<int, int> Get_EdgeEnds(int edge_idx);

        /**
         * @brief Get all edges incident to given vertex
         * 
         * @param vertex_id 
         * @return std::vector<Z> 
         */
        std::vector<Z> Get_IncidentEdges(int vertex_id);


        /**
         * @brief Get all vertices adjacent to given vertex
         * 
         * @param vertex_id 
         * @return std::vector<Z> 
         */
        std::vector<T> Get_AdjacentVertices(int vertex_id);


        /**
         * @brief Returns one of the vertices adjacent (as specified by the adjacent_id) to 'vertex_id' 
         * 
         * @param vertex_id 
         * @param adjacent_id 
         * @return T 
         */
        T Get_AdjacentVertex(int vertex_id, int adjacent_id);



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
         * @brief Makes an edge connection from Vertex 1 to Vertex 2 in the Graph.
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
        void Update_VertexData(int vertex_id, T data);


        /**
         * @brief Updates (overwrites) the edge data (from id1 to id2 for directed & between both for undirected) 
         * 
         * @param idx1 index of vertex whose edge will be updated
         * @param idx2 index of vertex whose edge will be updated
         * @param data new data
         */
        void Update_EdgeData(int idx1, int idx2, Z data);


        /**
         * @brief Performs Depth First Traversal of the Graph. (Recursively)
         * 
         * @param current_vertex 
         * @param visited - Array of visited vertices in the graph
         */
        void DFS(int current_vertex, vector<int> visited);


        /**
         * @brief Performs Breadth First Traversal of the Graph.
         * 
         * @param current_vertex 
         */
        void BFS(int current_vertex);


        /**
         * @brief Performs Depth First Traversal of the Graph. (Iteratively)
         * 
         * @param current_vertex 
         */
        void DFS_Iterative(int current_vertex);
        
    
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

// Definitions Below--------------------------------------------------------------------------------------------------




























































// Private------------------------------------------------------------------------------------------------------
template <typename T, typename Z>
bool Graph<T, Z>::beenVisited(int vertex, std::vector<int> visited) {
    
    for (int i = 0; i < visited.size(); i++) {

        if (vertex == visited[i]) {
            return true;
        }          
    } 

    return false;   
}

template <typename T, typename Z>
void Graph<T, Z>::Delete_Edge(int id) {

    typename std::vector<Edge<Z>>::iterator it = Edges.begin();
    advance(it, id);
    Edges.erase(it);
    Update_EdgeIDs();
}


template <typename T, typename Z>
void Graph<T, Z>::Update_EdgeIDs() {

    for (int i = 0; i < Edges.size(); i++) {
        
        if (Edges[i].edge_id != i) {

            Update_EdgeIDHelper(Edges[i].edge_id, Edges[i].vertexA, i);
            Update_EdgeIDHelper(Edges[i].edge_id, Edges[i].vertexB, i);
            Edges[i].edge_id = i;
        }
    }
}

template <typename T, typename Z>
void Graph<T, Z>::Update_EdgeIDHelper(int old_edge_id, int vertex_id, int new_edge_id) {

    for (int i = 0; i < Vertices[vertex_id].edges.size(); i++) {

        if (Vertices[vertex_id].edges[i] == old_edge_id) {

            Vertices[vertex_id].edges[i] = new_edge_id;
            break;
        }
    }
}

// Public----------------------------------------------------------------------------------------------------
template <typename T, typename Z>        
Graph<T, Z>::Graph() {

    Directed = NONE;
    RecentVertexID = 0;
}


template <typename T, typename Z>        
Graph<T, Z>::Graph(int state) : Directed(state) {

    RecentVertexID = 0;
}


template <typename T, typename Z>  
void Graph<T, Z>::SetGraphType(int state) {

    Directed = state;
}



template <typename T, typename Z>
bool Graph<T, Z>::isDirected() {

    return Directed > 0;
}


template <typename T, typename Z>
int Graph<T, Z>::Get_Degree(int vertex_id) {

    return Vertices[vertex_id].adjacents.size();
}



template <typename T, typename Z>
int Graph<T, Z>::Get_NumOfVertices() {

    return Vertices.size();
}

template <typename T, typename Z>
int Graph<T, Z>::Get_NumOfEdges() {

    return Edges.size();
}



template <typename T, typename Z>
T Graph<T, Z>::Get_Vertex(int vertex_id) {

    if (vertex_id < 0 || vertex_id >= Vertices.size()) {
        std::cout << "Error: This Vertex Index [" << vertex_id << "] doesn't exist." << std::endl;
    }
    
    return Vertices[vertex_id].data;
}

template <typename T, typename Z>
Z Graph<T, Z>::Get_Edge(int vertex_id, int edge_idx) {

    if (edge_idx < 0 || edge_idx >= Vertices[vertex_id].edges.size()) {
        std::cerr << "Error: Edge Index " << edge_idx << " is Invalid. Cannot get Edge." << std::endl;
    }

    if (vertex_id < 0 || vertex_id >= Vertices.size()) {
        std::cerr << "Error: Vertex Index " << vertex_id << " is Invalid. Cannot get Edge." << std::endl;
    }

    int index = Vertices[vertex_id].edges[edge_idx];
    return Edges[index].weight;
}


template <typename T, typename Z>
Z Graph<T, Z>::Get_EdgeByIndex(int index) {

    return Edges[index].weight;
}



template <typename T, typename Z>
Z Graph<T, Z>::Get_EdgeBetweenVertices(int idx1, int idx2) {

    if ((idx1 < 0 || idx1 >= Vertices.size())) {

        std::cerr << "Error: Vertex 1 Index " << idx1 << " is Invalid. Cannot get Edge." << std::endl;
    }

    if ((idx2 < 0 || idx2 >= Vertices.size())) {

        std::cerr << "Error: Vertex 2 Index " << idx2 << " is Invalid. Cannot get Edge." << std::endl;
    }

    if (idx1 == idx2) {

        std::cerr << "Error: Vertex 1 Index " << idx1 << " = Vertex 2 Index " << idx2 << std::endl;
    }

    for (int i = 0; i < Vertices[idx1].adjacents.size(); i++) {

        int edge_idx = Vertices[idx1].adjacents[i];

        if (Edges[edge_idx].vertexA == idx2 || Edges[edge_idx].vertexB == idx2)
            return Edges[edge_idx].weight;
    }
}



template <typename T, typename Z>
std::pair<int, int> Graph<T, Z>::Get_EdgeEnds(int edge_idx) {

    if (edge_idx < 0 || edge_idx >= Edges.size()) {
        std::cerr << "Error: Edge Index " << edge_idx << " is Invalid. Cannot get Edge." << std::endl;
    }

    return std::make_pair(Edges[edge_idx].vertexA, Edges[edge_idx].vertexB);
}


template <typename T, typename Z>
std::vector<Z> Graph<T, Z>::Get_IncidentEdges(int vertex_id) {

    std::vector<Z> incident_list;
    for (int i = 0; i < Vertices[vertex_id].edges.size(); i++) {

        int edge_id = Vertices[vertex_id].edges[i];
        //std::cout << "Edge ID: " << edge_id << " Num of Edges: " << Edges.size() << std::endl;
        incident_list.push_back(Edges[edge_id].weight);
    }
    return incident_list;
}


template <typename T, typename Z>
std::vector<T> Graph<T, Z>::Get_AdjacentVertices(int vertex_id) {

    std::vector<T> adjacent_list;
    for (int i = 0; i < Vertices[vertex_id].adjacents.size(); i++) {
        int adjacent_id = Vertices[vertex_id].adjacents[i];
        adjacent_list.push_back(Vertices[adjacent_id].data);
    }
    return adjacent_list;
}



template <typename T, typename Z>
T Graph<T, Z>::Get_AdjacentVertex(int vertex_id, int adjacent_id) {

    if (adjacent_id < 0 || adjacent_id >= Vertices[vertex_id].adjacents.size()) {
        std::cerr << "Error: Adjacent Index " << adjacent_id << " is Invalid. Cannot get Vertex." << std::endl;
    }

    if (vertex_id < 0 || vertex_id >= Vertices.size()) {
        std::cerr << "Error: Vertex Index " << vertex_id << " is Invalid. Cannot get Vertex." << std::endl;
    }

    int index = Vertices[vertex_id].adjacents[adjacent_id];
    return Vertices[index].data;
}



template <typename T, typename Z>
T Graph<T, Z>::Add_Vertex(T data, bool connected, Z weight) {

    Vertex<T, Z> new_vertex;
    new_vertex.data = data;
    new_vertex.id = Vertices.size();
    new_vertex.err = 0;
    Vertices.push_back(new_vertex);

    if (connected) {

        Add_Edge(RecentVertexID, new_vertex.id, weight);
    }

    RecentVertexID = new_vertex.id;
    return new_vertex.data;
}


template <typename T, typename Z>
void Graph<T, Z>::Add_Edge(int vertex1_ID, int vertex2_ID, Z weight) {

    if ((vertex1_ID < 0 || vertex1_ID > Vertices.size() - 1)) {

        std::cerr << "Error: Vertex 1 Index " << vertex1_ID << " is Invalid. Cannot add Edge." << std::endl;
        return;
    }

    if ((vertex2_ID < 0 || vertex2_ID > Vertices.size() - 1)) {

        std::cerr << "Error: Vertex 2 Index " << vertex2_ID << " is Invalid. Cannot add Edge." << std::endl;
        return;
    }

    
    Edge<Z> new_edge;
    new_edge.edge_id = Edges.size();
    new_edge.vertexA = vertex1_ID;
    new_edge.vertexB = vertex2_ID;
    new_edge.weight = weight;

    // Edge from V1 to V2
    if (Directed == FORWARD) {
        Vertices[vertex1_ID].edges.push_back(new_edge.edge_id);
        Vertices[vertex1_ID].adjacents.push_back(vertex2_ID);
    }

    // Edge from V2 to V1
    if (Directed == BACKWARD) {
        Vertices[vertex2_ID].edges.push_back(new_edge.edge_id);
        Vertices[vertex2_ID].adjacents.push_back(vertex1_ID);
    }
    
    // Edge from V1 to V2 AND V2 to V1
    if (Directed == NONE) {
        Vertices[vertex1_ID].edges.push_back(new_edge.edge_id);
        Vertices[vertex1_ID].adjacents.push_back(vertex2_ID);
        Vertices[vertex2_ID].edges.push_back(new_edge.edge_id);
        Vertices[vertex2_ID].adjacents.push_back(vertex1_ID);
    }

    Edges.push_back(new_edge);
}


// TODO: Not Finished
template <typename T, typename Z>
void Graph<T, Z>::Remove_Edge(int index1, int index2) {

    if ((index1 < 0 || index1 > Vertices.size() - 1)) {

        std::cerr << "Error: Vertex 1 Index " << index1 << " is Invalid. Cannot remove Edge." << std::endl;
        return;
    }

    if ((index2 < 0 || index2 > Vertices.size() - 1)) {

        std::cerr << "Error: Vertex 2 Index " << index2 << " is Invalid. Cannot remove Edge." << std::endl;
        return;
    }

    int edge_index = -1;
    typename std::vector<int>::iterator edge_iter1 = Vertices[index1].edges.begin();
    typename std::vector<int>::iterator edge_iter2 = Vertices[index2].edges.begin();
    typename std::vector<int>::iterator adjacent_iter1 = Vertices[index1].adjacents.begin();
    typename std::vector<int>::iterator adjacent_iter2 = Vertices[index2].adjacents.begin();

    // Remove Edge from Index 1 Side
    for (int i = 0; i < Vertices[index1].adjacents.size(); i++) {

        if (Vertices[index1].adjacents[i] == index2) {

            edge_index = *edge_iter1;
            Vertices[index1].edges.erase(edge_iter1);
            Vertices[index1].adjacents.erase(adjacent_iter1);
            break;
        }
        else {
            edge_iter1++;
            adjacent_iter1++;
        }
    }

    // Remove Edge from Index 2 Side
    for (int i = 0; i < Vertices[index2].adjacents.size(); i++) {

        if (Vertices[index2].adjacents[i] == index1) {

            edge_index = *edge_iter2;
            Vertices[index2].edges.erase(edge_iter2);
            Vertices[index2].adjacents.erase(adjacent_iter2);
            break;
        }
        else {
            edge_iter2++;
            adjacent_iter1++;
        }    
    }

    if (edge_index > -1) { Delete_Edge(edge_index); }
}


template <typename T, typename Z>
void Graph<T, Z>::Update_VertexData(int vertex_id, T data) {

    if (vertex_id < 0 || vertex_id >= Vertices.size()) {
        std::cout << "This Index doesn't exist." << std::endl;
        return;
    }
    
    Vertices[vertex_id].data = data;
}



template <typename T, typename Z>
void Graph<T, Z>::Update_EdgeData(int idx1, int idx2, Z data) {

    if ((idx1 < 0 || idx1 >= Vertices.size())) {

        std::cerr << "Error: Vertex 1 Index " << idx1 << " is Invalid. Cannot update Edge." << std::endl;
    }

    if ((idx2 < 0 || idx2 >= Vertices.size())) {

        std::cerr << "Error: Vertex 2 Index " << idx2 << " is Invalid. Cannot update Edge." << std::endl;
    }

    // Update edge
    for (int i = 0; i < Vertices[idx1].adjacents.size(); i++) {
        int edge_idx1 = Vertices[idx1].edges[i];
        if (Vertices[idx1].adjacents[i] == idx2) {
            Edges[edge_idx1].weight = data;
            break;
        }
    }
}


template <typename T, typename Z>
void Graph<T, Z>::DFS(int current_vertex, std::vector<int> visited) {

    if (current_vertex < 0 || current_vertex > Vertices.size()){
        std::cerr << "Invalid Vertex ID" << std::endl;
    }

    // Add vertex to visited list
    visited.push_back(current_vertex);

    // Loop through each adjacent vertex
    for (int i = 0; i < Vertices[current_vertex].adjacents.size(); i++) {

        int adjacent = Vertices[current_vertex].adjacents[i];

        // If adjacent has not visited, continue depth search into adjacent
        if (!beenVisited(adjacent, visited)) 
            DFS(adjacent, visited);
    }    
}


template <typename T, typename Z>
void Graph<T, Z>::BFS(int current_vertex) {

    Vertex<T, Z> error;

    if (current_vertex < 0 || current_vertex > Vertices.size()){
        std::cerr << "Invalid Vertex ID" << std::endl;
    }

    std::vector<int> visited;
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
        for (int i = 0; i < Vertices[vertex].adjacents.size(); i++) {

            int adjacent = Vertices[vertex].adjacents[i];

            // If adjacent has not visited, mark it as visited and enqueue it
            if (!beenVisited(adjacent, visited)) {    
                
                visited.push_back(adjacent);
                task_queue.push(adjacent);
            }
        }
    }
}



template <typename T, typename Z>
void Graph<T, Z>::DFS_Iterative(int current_vertex) {


    if (current_vertex < 0 || current_vertex > Vertices.size()){
        std::cerr << "Invalid Vertex ID" << std::endl;
    }

    std::stack<int> branch_stack;
    branch_stack.push(current_vertex);
    std::vector<int> visited;

    while(branch_stack.size() > 0) {

        int vertex = branch_stack.pop();

        // If vertex has not visited
        if (!beenVisited(vertex, visited)) {
            visited.push_back(vertex);

            // Check all adjacent neighbors
            for (int i = 0; i < Vertices[vertex].adjacents.size(); i++) {
                int neighbor = Vertices[vertex].adjacents[i];

                // If neighbor has not been visited.
                if (!beenVisited(neighbor, visited))
                    branch_stack.push(neighbor);
            }
        }
    } 
}


template <typename T, typename Z>
void Graph<T, Z>::Print_Vertices() {

    for (int i = 0; i < Vertices.size(); i++) {

        std::cout << Vertices[i].id << " -> " << Vertices[i].data << std::endl;
    }
}



template <typename T, typename Z>
void Graph<T, Z>::Print_Edges() {
    
    for (int i = 0; i < Vertices.size(); i++) {

        for (int j = 0; j < Vertices[i].edges.size(); j++) {

            std::cout << Vertices[i].id << "- " << Vertices[i].edges[j].vertexA << "<->" << Vertices[i].edges[j].vertexB << "  ";
        }
        
        std::cout << "\n";
    }
}



/*
 * 			TO-DO
 * 			-----
 *  - 
 *
 *  - 
 *  
 *  - 
 *  */