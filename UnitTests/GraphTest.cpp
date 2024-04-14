#include <iostream>
#include <gtest/gtest.h>
#include "Graph.hpp"


class GraphTest : public ::testing::Test {
  
  protected:
    
    GraphTest() {
      
      // Test Set-Up
      graph = new Graph<int, int>(vertices, false);
    }

    //~GraphTest() {/*Test Tear Down after each test*/}

    std::vector<Vertex<int,int>> vertices;
    Graph<int, int> *graph;
};

/**
 * @brief 
 *
 * **/
TEST_F(GraphTest, isDirectedTest) {

  EXPECT_FALSE(graph->isDirected()) << "Graph status does not match expected outcome";
}

/**
 * @brief 
 *
 * **/
TEST_F(GraphTest, VertexManipulation1) {
  
  // Test Add_Vertex & Get_NumOfVertices
  graph->Add_Vertex(30, false, 0);
  graph->Add_Vertex(20, false, 0);
  graph->Add_Vertex(45, false, 0);
  EXPECT_EQ(graph->Get_NumOfVertices(), 3);

  // Test Get_Vertex
  EXPECT_EQ(graph->Get_Vertex(0), 30);
  EXPECT_EQ(graph->Get_Vertex(2), 45);
}

/**
 * @brief 
 *
 * **/
TEST_F(GraphTest, ConnectionsTest) {
  
  graph->Add_Vertex(30, false, 0);
  graph->Add_Vertex(20, false, 0);
  graph->Add_Vertex(45, false, 0);

  graph->Add_Edge(0, 2, 50);
  std::vector<Edge<int>> edges = graph->Get_AdjacentVertices(0);
  EXPECT_EQ(edges.front().AdjacentVertexID, 2);
}

/**
 * @brief 
 *
 * **/
TEST_F(GraphTest, Connect_NewVertexTest) {
  
  graph->Add_Vertex(30, false, 0);
  graph->Add_Vertex(20, false, 0);
  graph->Add_Vertex(45, false, 0);

  graph->Add_Edge(100, 1, 37);
  std::vector<Edge<int>> edges = graph->Get_AdjacentVertices(1);
  EXPECT_EQ(edges.front().AdjacentVertexID, 3);
}

/**
 * @brief 
 *
 * **/
TEST_F(GraphTest, Print_VerticesTest) {
  
  graph->Add_Vertex(30, false, 0);
  graph->Add_Vertex(20, false, 0);
  graph->Add_Vertex(45, false, 0);
  graph->Add_Edge(0, 2, 50);
  graph->Add_Edge(0, 1, 50);
  graph->Add_Edge(1, 2, 50);

  // Version 1
  testing::internal::CaptureStdout();
  graph->Print_Vertices();
  std::string output = testing::internal::GetCapturedStdout();

  /*
  // Version 2
  // This can be an ofstream as well or any other ostream
  std::stringstream buffer;
  // Save cout's buffer here
  std::streambuf *sbuf = std::cout.rdbuf();
  // Redirect cout to our stringstream buffer or any other ostream
  std::cout.rdbuf(buffer.rdbuf());
  // Use cout as usual
  graph->Print_Vertices();
  // When done redirect cout to its old self
  std::cout.rdbuf(sbuf);
  */
}

/**
 * @brief 
 *
 * **/
TEST_F(GraphTest, Print_EdgesTest) {
  
  graph->Add_Vertex(30, false, 0);
  graph->Add_Vertex(20, false, 0);
  graph->Add_Vertex(45, false, 0);
  graph->Add_Edge(0, 2, 50);
  graph->Add_Edge(0, 1, 50);
  graph->Add_Edge(1, 2, 50);

  testing::internal::CaptureStdout();
  graph->Print_Edges();
  std::string output = testing::internal::GetCapturedStdout();
}

/**
 * @brief 
 *
 * **/
TEST_F(GraphTest, DFSTest) {

  graph->Add_Vertex(30, false, 0);
  graph->Add_Vertex(20, false, 0);
  graph->Add_Vertex(45, false, 0);
  graph->Add_Edge(0, 2, 50);
  graph->Add_Edge(0, 1, 50);
  graph->Add_Edge(1, 2, 50);

  vector<int> visited;
  Vertex<int, int> v_result = graph->DFS(45, 0, visited); 
  EXPECT_NE(v_result.Data, NULL);
  EXPECT_EQ(v_result.Data, 45);
}

/**
 * @brief 
 *
 * **/
TEST_F(GraphTest, BFSTest) {

  graph->Add_Vertex(30, false, 0);
  graph->Add_Vertex(20, false, 0);
  graph->Add_Vertex(45, false, 0);
  graph->Add_Edge(0, 2, 50);
  graph->Add_Edge(0, 1, 50);
  graph->Add_Edge(1, 2, 50);
  
  Vertex<int, int> v_result = graph->BFS(45, 0); 
  EXPECT_NE(v_result.Data, NULL);
  EXPECT_EQ(v_result.Data, 45);
}








