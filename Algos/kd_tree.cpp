#include "kd_tree.hpp"

// Private ------------------------------------------------------------------------
float KDTree::get_radius_squared(VectorXf point_a, VectorXf point_b) {
    
    float dist = 0.f;
    for (int i = 0; i < point_a.rows(); i++) {

        float diff = std::abs(point_a[i] - point_b[i]);
        dist += (diff * diff);   
    } 

    return dist;
}

struct Node * KDTree::get_closest(VectorXf point, Node *current_node, Node *temp) {

    if (current_node == nullptr) {
        return temp;
    }

    if (temp == nullptr) {
        return current_node;
    }

    float current_rad = get_radius_squared(point, current_node->data);
    float temp_rad = get_radius_squared(point, temp->data);

    return (current_rad < temp_rad)? current_node : temp;
}

bool KDTree::point_match(VectorXf point_a, VectorXf point_b) {

    for (int i = 0; i < point_a.rows(); i++) {

        if (point_a[i] != point_b[i]) { return false; }
    }

    return true;
}


// Public -------------------------------------------------------------------------
KDTree::KDTree() {}


KDTree::KDTree(int k) : k(k) {

}


struct Node* KDTree::build_tree(std::vector<VectorXf> points) {

    // Build new tree
    kd_tree = new struct Node;
    kd_tree->left = nullptr;
    kd_tree->right = nullptr;

    // Initialize the root
    kd_tree->data = points[0];
    kd_tree->pos = 0;

    for (int i = 1; i < points.size(); i++) {

        insert(points[i], i, kd_tree, 0);
    }

    return kd_tree;
}


void KDTree::insert(VectorXf point, int pos, struct Node *current_node, int depth) {

    // Go Left
    if (point[depth % k] < current_node->data[depth % k]) {

        if (current_node->left == nullptr) {
    
            struct Node *newNode = new Node;
            newNode->data = point;
            newNode->pos = pos;
            newNode->left = nullptr;
            newNode->right = nullptr;
            current_node->left = newNode;
            return;
        }

        else {
             insert(point, pos, current_node->left, depth + 1);
        }
    }

    // Go Right
    else if (point[depth % k] >= current_node->data[depth % k]) {

        if (current_node->right == nullptr) {
    
            struct Node *newNode = new Node; 
            newNode->data = point;
            newNode->pos = pos;
            newNode->left = nullptr;
            newNode->right = nullptr;
            current_node->right = newNode;
            return;
        }

        else {
             insert(point, pos, current_node->right, depth + 1);
        } 
    }
}


struct Node* KDTree::remove(VectorXf point, struct Node *current_node, int depth) {

    if (current_node == nullptr) { return NULL; }

    // Match
    else if (point[depth % k] == current_node->data[depth % k]) {

        if (point_match(point, current_node->data)) {
        
            // If leaf node
            if (current_node->left == nullptr && current_node->right == nullptr) {
                delete current_node;
                return NULL;
            }

            else if (current_node->right != nullptr) {  
                // Replace node to delete with min
                struct Node *min = find_min(current_node->right);
                current_node->data = min->data;
                current_node->pos = min->pos;

                // Delete the Min
                current_node->right = remove(min->data, current_node->right, depth + 1);
            }

            else if (current_node->left != nullptr) {
                // Replace node to delete with min
                struct Node *min = find_min(current_node->left);
                current_node->data = min->data;
                current_node->pos = min->pos;
                
                // Delete the Min
                current_node->right = remove(min->data, current_node->left, depth + 1);
            }

            return current_node;
        }

        else {
            current_node->right = remove(point, current_node->right, depth + 1);  
        }
    }

    // Go Left
    else if (point[depth % k] < current_node->data[depth % k]) {
        
        current_node->left = remove(point, current_node->left, depth + 1); 
    }

    // Go Right
    else if (point[depth % k] > current_node->data[depth % k]) {

        current_node->right = remove(point, current_node->right, depth + 1);  
    }

    return current_node;
}


struct Node* KDTree::find_min(struct Node *current_node) {

    if (current_node->left == nullptr) { return current_node; }
    return find_min(current_node->left);
}


struct Node* KDTree::search(VectorXf point, struct Node *current_node, int depth) {

    struct Node *found = NULL;

    if (current_node == nullptr) { return NULL; }

    // Match
    else if (point[depth % k] == current_node->data[depth % k]) {

        if (point_match(point, current_node->data)) {

            found = current_node;
        }

        else {
            found = search(point, current_node->right, depth + 1);  
        }
    }

    // Go Left
    else if (point[depth % k] < current_node->data[depth % k]) {

        found = search(point, current_node->left, depth + 1);
    }

    // Go Right
    else if (point[depth % k] > current_node->data[depth % k]) {

        found = search(point, current_node->right, depth + 1);
    }

    return found;
}


struct Node* KDTree::get_nearest_neighbor(VectorXf point, struct Node *current_node, int depth) {

    struct Node *nextBranch = NULL;
    struct Node *otherBranch = NULL;

    if (current_node == nullptr) { return nullptr; }

    if (point[depth % k] < current_node->data[depth % k]) {
        nextBranch = current_node->left;
        otherBranch = current_node->right;
    }
    else {
        nextBranch = current_node->right;
        otherBranch = current_node->left;
    }

    struct Node *temp = get_nearest_neighbor(point, nextBranch, depth + 1);
    struct Node *nearest = get_closest(point, current_node, temp);

    float radius_squared = get_radius_squared(point, nearest->data);
    float dist = point[depth % k] - current_node->data[depth % k];

    if (radius_squared >= (dist * dist)) {
        temp = get_nearest_neighbor(point, otherBranch, depth + 1);
        nearest = get_closest(point, current_node, temp);
    } 

    return nearest;
}











