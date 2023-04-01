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

        Connect_Vertices(RecentVertexID, new_vertex.VertexID, weight);
    }

    RecentVertexID = G.size();
    return new_vertex;
}


template <typename T, typename Z>
void Graph<T, Z>::Connect_Vertices(int vertex1_ID, int vertex2_ID, Z weight) {

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
void Graph<T, Z>::Disconnect_Vertices(int index1, int index2) {

    if ((vertex1_ID < 0 || vertex1_ID > G.size()) || (vertex2_ID < 0 || vertex2_ID > G.size())) {

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
Vertex<T, Z> Graph<T, Z>::DFS(T target_data, int current_vertex, vector<Vertex<T, Z>> visited_stack) {

    // Implement Code
    Vertex<T, Z> starting_vertex;
    //vector<Vertex<T, Z>> VisitedStack;

    if (G[i].Data == target_data) {

        return G[i];
    }

    visited_stack.push_back(G[current_vertex]);
    DFS(target_data, G[current_vertex].Adjacents[G[current_vertex].adjacents_pointer], visited_stack);

    

    for (int i = 0; i < G.size(); i++) {

        


        if (G[i].Adjacents[G[i].adjacents_pointer] == target_data) {

            return G[i];
        }

        G[i].adjacents_pointer++;
        vector<Edge<Z>> adjacents = Get_AdjacentVertices(G[i].VertexID);
        visited_stack.push_back(G[i].Adjacents[G[i].adjacents_pointer]);


    }
    
    return NULL;
}


template <typename T, typename Z>
Vertex<T, Z> Graph<T, Z>::BFS(T target_data) {

    // Implement Code
    vector<Vertex<T, Z>> VisitedQueue;
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