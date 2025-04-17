#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <queue>
using std::vector;
using namespace::Eigen;
struct Packet {
    VectorXf data{0};
    float weight;
};

struct Node {
    Packet packet;
    Node *left;
    Node *right;
};


class KDTree {

    private:
        Node *kd_tree;
        int k;
        // std::vector<Packet> raw_points;

        /**
         * @brief Get the Depth object
         * 
         * @param node 
         * @return int 
         */
        int getDepth(Node* node);
        
        /**
         * @brief 
         * 
         * @param node 
         * @param points 
         */
        void collectPoints(Node* node, std::vector<VectorXf> &points);
        
        /**
         * @brief 
         * 
         * @param points 
         * @param depth 
         * @return Node* 
         */
        Node* buildBalancedTree(std::vector<VectorXf> &points, int depth);

        /**
         * @brief Retruns the squared distance between two points
         * 
         * @param point_a 
         * @param point_b 
         * @return float 
         */
        float get_radius_squared(VectorXf point_a, VectorXf point_b);

        /**
         * @brief Determine which node (node a or node b) is closest to the given point
         * 
         * @param point 
         * @param node_a 
         * @param node_b 
         * @return Node* 
         */
        Node * get_closest(VectorXf point, Node *node_a, Node *node_b);

        /**
         * @brief 
         * 
         * @param current_node 
         * @return Node* 
         */
        Node* findMin(Node *current_node);

        /**
         * @brief 
         * 
         * @param point_a 
         * @param point_b 
         * @return true 
         * @return false 
         */
        bool point_match(VectorXf point_a, VectorXf point_b);

        /**
         * @brief Insert new node into the KD Tree
         * 
         * @param point 
         * @param current_node 
         * @param depth 
         * @return Node* 
         */
        Node* insert(VectorXf point, Node *current_node, int depth);

        /**
         * @brief 
         * 
         * @param point 
         * @param current_node 
         * @param depth 
         * @return Node* Root of the modified tree
         */
        Node* remove(VectorXf point, Node *current_node, int depth);

        /**
         * @brief 
         * 
         * @param point 
         * @param current_node 
         * @param depth 
         * @return Node* 
         */
        Node* search(VectorXf point, Node *current_node, int depth);

        /**
         * @brief 
         * 
         * @param point 
         * @param current_node 
         * @param depth 
         * @return Node* 
         */
        Node* get_nearest_neighbor(VectorXf point, Node *current_node, int depth);

        

    public:
        std::vector<Packet> raw_points;

        /**
         * @brief Construct a new KDTree object
         * 
         */
        KDTree();

        /**
         * @brief Construct a new KDTree object
         * 
         * @param k 
         */
        KDTree(int k);

        /**
         * @brief 
         * 
         * @return Node* 
         */
        Node* GetKDTree();

        /**
         * @brief 
         * 
         * @param point 
         */
        void Insert(VectorXf point);
        
        /**
         * @brief 
         * 
         * @param point 
         */
        void Remove(VectorXf point);
        
        /**
         * @brief 
         * 
         * @param point 
         * @return VectorXf 
         */
        Packet Search(VectorXf point);

        /**
         * @brief 
         * 
         * @param point 
         * @return VectorXf 
         */
        Packet NearestNeighbor(VectorXf point);


        /**
         * @brief 
         * 
         * @param point 
         */
        void AddData(VectorXf point, float weight);

        
        /**
         * @brief 
         * 
         */
        void buildKDTree(); 

};