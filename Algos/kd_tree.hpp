#pragma once
#include <iostream>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>

using std::vector;
using namespace::Eigen;

struct Node {
    VectorXf data;
    int pos;
    struct Node *left;
    struct Node *right;
};


class KDTree {

    private:
        struct Node *kd_tree;
        int k;

        /**
         * @brief Retruns the squared distance between two points
         * 
         * @param point_a 
         * @param point_b 
         * @return float 
         */
        float get_radius_squared(VectorXf point_a, VectorXf point_b);

        /**
         * @brief 
         * 
         * @param point 
         * @param current_node 
         * @param temp 
         * @return struct Node* 
         */
        struct Node * get_closest(VectorXf point, Node *current_node, Node *temp);

        /**
         * @brief 
         * 
         * @param point_a 
         * @param point_b 
         * @return true 
         * @return false 
         */
        bool point_match(VectorXf point_a, VectorXf point_b);

        

    public:

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
         * @param points 
         * @return struct Node* 
         */
        struct Node* build_tree(std::vector<VectorXf> points);

        /**
         * @brief insert new node into the KD Tree
         * 
         * @param point 
         * @param pos 
         * @param current_node 
         * @param depth 
         */
        void insert(VectorXf point, int pos, struct Node *current_node, int depth);

        /**
         * @brief 
         * 
         * @param point 
         * @param current_node 
         * @param depth 
         * @return struct Node* Root of the modified tree
         */
        struct Node* remove(VectorXf point, struct Node *current_node, int depth);

        /**
         * @brief 
         * 
         * @param current_node 
         * @return struct Node* 
         */
        struct Node* find_min(struct Node *current_node);

        /**
         * @brief 
         * 
         * @param point 
         * @param current_node 
         * @param depth 
         * @return struct Node* 
         */
        struct Node* search(VectorXf point, struct Node *current_node, int depth);

        /**
         * @brief 
         * 
         * @param point 
         * @param current_node 
         * @param depth 
         * @return struct Node* 
         */
        struct Node* get_nearest_neighbor(VectorXf point, struct Node *current_node, int depth);
};