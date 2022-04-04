#include "graph.hpp"


template <typename T, typename Z>
Vertex<T, Z>::Vertex() {

}



template <typename T, typename Z>
Vertex<T, Z>::Vertex(T data) {
    
    Data = data;
    NumOfNeighbors = 0;
}



template <typename T, typename Z>
void Vertex<T, Z>::Add_Data(T data) {
    Data = data;
}



template <typename T, typename Z>
T Vertex<T, Z>::Get_Data() {
    return Data;
}



template <typename T, typename Z>
void Vertex<T, Z>::Connect_Vertex(Vertex<T, Z> newVertex, Z weight) {
    
    Neighbor N;
    N.Vertex<T> = newVertex;
    N.Edge_Weight = weight;
    Neighbors[NumOfNeighbors - 1]= N;

    NumOfNeighbors++;
}



template <typename T, typename Z>
void Vertex<T, Z>::Print_Edges() {
    
    for (int i = 0; i < NumOfNeighbors; i++) {

        std::cout << Neighbors[i].Vertex_ID << std::endl;
    }
}



template <typename T, typename Z>
void Vertex<T, Z>::Set_VertexID(int ID) {
    VertexSet_Index = ID;
}



template <typename T, typename Z>
int Vertex<T, Z>::Get_VertexID() {
    return VertexSet_Index;
}



template <typename T, typename Z>
void Vertex<T, Z>::Remove_Edge(int vertexID) {

	for (int i = 0; i < Neighbors.size(); i++) {
		
		if (vertexID = Neighbors[i].NeighborVertex.Get_VertexID()) {
			
			Neighbors.erase(Neighbors.begin() + i);
		}
	}
}

//-------------------------------------------------------------------------------
template <typename T, typename Z>
Graph<T, Z>::Graph() {

}



template <typename T, typename Z>
Graph<T, Z>::Graph(bool state) {
    
    directed = state;
    VSet.first = 0; // Number of Vertices
}



template <typename T, typename Z>
bool Graph<T, Z>::isDirected() {
    
    return directed;
}



template <typename T, typename Z>
int Graph<T, Z>::Get_NumOfVertices() {
    
    return VSet.first;
}



template <typename T, typename Z>
T Graph<T, Z>::Get_Vertex(int vertex_ID) {
    
    if (vertex_ID < 0 || vertex_ID >= VSet.first) {
        std::cout << "This Index doesn't exist." << std::endl;
        return;
    }
    
    return VSet.second[vertex_ID].Get_Data();
}



template <typename T, typename Z>
void Graph<T, Z>::Add_Vertex(T data) {
    
    Vertex new_vertex(data);
    new_vertex.Set_VertexID(VSet.first);
    VSet.second[VSet.first] = new_vertex;
    VSet.first++;
}



template <typename T, typename Z>
void Graph<T, Z>::Make_Connection(int vertex1_ID, int vertex2_ID, Z weight) {
    
    // Edge from V1 to V2
    VSet.second[vertex1_ID].Connect_Vertex(VSet.second[vertex2_ID], weight);

    // Edge from V2 to V1 (If Undirected)
    if (!directed) {
        VSet.second[vertex2_ID].Connect_Vertex(VSet.second[vertex1_ID], weight);
    }
}



template <typename T, typename Z>
void Graph<T, Z>::Connect_NewVertex(int vertexConnect_ID, T new_vertex_data, Z weight) {
    
    Add_Vertex(new_vertex_data);
    VSet.second[vertexConnect_ID].Connect_Vertex(VSet.second[VSet.first - 1], weight);

    if (!directed) {
        VSet.second[VSet.first - 1].Connect_Vertex(VSet.second[vertexConnect_ID], weight);
    }
}



template <typename T, typename Z>
pair<int, vector<Vertex<T, Z>>> Graph<T, Z>::Get_Vertices() {
    
    return VSet;
}



template <typename T, typename Z>
void Graph<T, Z>::Print_Vertices() {

    for (int i = 0; i < VSet.first; i++) {

        std::cout << VSet.second[i].Get_Data() << std::endl;
    }
}



template <typename T, typename Z>
void Graph<T, Z>::Print_Edges() {
    
    for (int i = 0; i < VSet.first; i++) {

        VSet.second[i].Get_Edges();
        std::cout << "\n";
    }
}



template <typename T, typename Z>
void Graph<T, Z>::Disconnect_Vertices(int index1, int index2) {

	Get_Vertex(index1).Remove_Edge(index2);
	Get_Vertex(index2).Remove_Edge(index1);

}



template <typename T, typename Z>
void Graph<T, Z>::DFS(Vertex<T, Z> starting_vertex, Vertex<T, Z> target_vertex, 
    vector<Vertex<T, Z>> Visited) {

    // Implement Code
}



template <typename T, typename Z>
void Graph<T, Z>::BFS(Vertex<T, Z> starting_vertex, Vertex<T, Z> target_vertex) {

    // Implement Code
    Vertex<T, Z> *path;
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
