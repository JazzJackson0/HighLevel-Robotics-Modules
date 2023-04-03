#include "Graph.hpp"

template <typename T, typename Z>        
Graph<T, Z>::Graph() {

    Directed = false;
    RecentVertexID = 0;
}


template <typename T, typename Z>        
Graph<T, Z>::Graph(vector<Vertex<T, Z>> adjacency_list, bool state) : G(adjacency_list) {

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
        std::cout << "This Index doesn't exist." << std::endl;
        return;
    }
    
    return G[vertex_id].Data;
}



template <typename T, typename Z>
vector<Vertex<T, Z>> Graph<T, Z>::Get_Graph() {
    
    return G;
}


template <typename T, typename Z>
vector<Edge<Z>> Graph<T, Z>::Get_AdjacentVertices(int vertex_id) {

    return G[vertex_id].Adjacents;
}


template <typename T, typename Z>
Vertex<T, Z> Graph<T, Z>::Add_Vertex(T data, bool connected, Z weight) {

    Vertex<T, Z> new_vertex;
    new_vertex.Data = data;
    new_vertex.VertexID = G.size();
    new_vertex.adjacents_pointer = 0;
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

    if ((index1 < 0 || index1 > G.size()) || (index2 < 0 || index2 > G.size())) {

        std::cerr << "Invalid Index" << std::endl;
        return;
    }

    typename std::list<Vertex<T, Z>>::iterator iter1;
    typename std::list<Vertex<T, Z>>::iterator iter2;
    iter1 = G[index1].begin();
    iter2 = G[index2].begin();

    // Remove Edge from Index 1 Side
    for (int i = 0; i < G[index1].size(); i++) {

        if (G[index1][i].AdjacentVertexID == index2) {

            G[index1][i].erase(iter1);
        }
        iter1++;
    }

    // Remove Edge from Index 2 Side
    for (int i = 0; i < G[index2].size(); i++) {

        iter2 = G[index2].begin();

        if (G[index2][i].AdjacentVertexID == index1) {

            G[index2][i].erase(iter2);
        }
        iter1++;
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
Vertex<T, Z> Graph<T, Z>::DFS(T target_data, int current_vertex, vector<int> visited) {

    if (current_vertex < 0 || current_vertex > G.size()){
        std::cerr << "Invalid Vertex ID" << std::endl;
        return NULL;
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
        if (!been_visited) {    
            been_visited = false;
            DFS(target_data, G[current_vertex].Adjacents[i].AdjacentVertexID, visited);
        }
    }

    // Can't go deeper
    if (visited == G.size())
        return NULL;     
}


template <typename T, typename Z>
Vertex<T, Z> Graph<T, Z>::BFS(T target_data, int current_vertex) {

    if (current_vertex < 0 || current_vertex > G.size()){
        std::cerr << "Invalid Vertex ID" << std::endl;
        return NULL;
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
        int vertex = task_queue.pop();

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
                been_visited = false;
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
        }
    }

    return NULL;
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

            std::cout << G[i].VertexID << "-" << G[i].Adjacents[j] << " ";
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