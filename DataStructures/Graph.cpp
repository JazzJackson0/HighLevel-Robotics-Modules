#include "Graph.hpp"

template <typename T, typename Z>        
Graph<T, Z>::Graph() {}


template <typename T, typename Z>        
Graph<T, Z>::Graph(std::vector<Vertex<T, Z>> &adjacency_list, bool state) : G(adjacency_list) {

    Directed = state;
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
T Graph<T, Z>::Get_Vertex(int vertex_ID) {

    if (vertex_ID < 0 || vertex_ID >= G.size()) {
        std::cout << "This Index doesn't exist." << std::endl;
        return;
    }
    
    return G[vertex_ID].Data;
}


template <typename T, typename Z>
void Graph<T, Z>::Add_Vertex(T data) {

    Vertex<T, Z> new_vertex;
    new_vertex.Data = data;
    new_vertex.Vertex_ID = G.size();
    G.push_back(new_vertex);
}


template <typename T, typename Z>
void Graph<T, Z>::Make_Connection(int vertex1_ID, int vertex2_ID, Z weight) {

    // Edge from V1 to V2
    Edge<Z> new_edge;
    new_edge.AdjacentVertex_ID = vertex2_ID;
    new_edge.Weight = weight;
    G[vertex1_ID].Adjacents.push_back(new_edge);

    // Edge from V2 to V1 (If Undirected)
    if (!Directed) {
        Edge<Z> new_edge2;
        new_edge2.AdjacentVertex_ID = vertex1_ID;
        new_edge2.Weight = weight;
        G[vertex2_ID].Adjacents.push_back(new_edge2);
    }
}

template <typename T, typename Z>
std::list<Edge<Z>> Graph<T, Z>::Get_AdjacentVertices(int vertex_ID) {

    return G[vertex_ID].Adjacents;
}


template <typename T, typename Z>
void Graph<T, Z>::Connect_NewVertex(T new_vertex_data, int vertexConnect_ID, Z weight) {

    Add_Vertex(new_vertex_data);
    Make_Connection(G.size() - 1, vertexConnect_ID, weight);
}


template <typename T, typename Z>
std::vector<Vertex<T, Z>> Graph<T, Z>::Get_Vertices() {
    
    return G;
}


template <typename T, typename Z>
void Graph<T, Z>::Print_Vertices() {

    for (int i = 0; i < G.size(); i++) {

        std::cout << G[i].Vertex_ID << " -> " << G[i].Data << std::endl;
    }
}



template <typename T, typename Z>
void Graph<T, Z>::Print_Edges() {
    
    for (int i = 0; i < G.size(); i++) {

        for (int j = 0; j < G[i].Adjacents.size(); j++) {

            std::cout << G[i].Vertex_ID << "-" << G[i].Adjacents[j] << " ";
        }
        
        std::cout << "\n";
    }
}



template <typename T, typename Z>
void Graph<T, Z>::Disconnect_Vertices(int index1, int index2) {

    typename std::list<Vertex<T, Z>>::iterator iter1;
    typename std::list<Vertex<T, Z>>::iterator iter2;
    iter1 = G[index1].begin();
    iter2 = G[index2].begin();

    // Remove Edge from Index 1 Side
    for (int i = 0; i < G[index1].size(); i++) {

        if (G[index1][i].AdjacentVertex_ID == index2) {

            G[index1][i].erase(iter1);
        }
        iter1++;
    }

    // Remove Edge from Index 2 Side
    for (int i = 0; i < G[index2].size(); i++) {

        iter2 = G[index2].begin();

        if (G[index2][i].AdjacentVertex_ID == index1) {

            G[index2][i].erase(iter2);
        }
        iter1++;
    }

}



template <typename T, typename Z>
Vertex<T, Z> Graph<T, Z>::DFS(T target_data) {

    // Implement Code
    //Vertex<T, Z> starting_vertex
    //Vertex<T, Z> target_data, 
    //std::vector<Vertex<T, Z>> Visited
    
    return NULL;
}



template <typename T, typename Z>
Vertex<T, Z> Graph<T, Z>::BFS(T target_data) {

    // Implement Code

    return NULL;
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