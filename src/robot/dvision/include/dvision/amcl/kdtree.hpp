#pragma once
#include <vector>
#include <array>
#include "dvision/amcl/pose.hpp"
namespace dvision {
// Info for a node in the tree
struct KdTreeNode {
    KdTreeNode() : leaf(true) {

    }

    bool operator==(const KdTreeNode& oth) const {
        return (key[0] == oth.key[0])
            && (key[1] == oth.key[1])
            && (key[2] == oth.key[2]);
    }

    bool keyEqual(int othKey[]) {
        return (key[0] == othKey[0])
            && (key[1] == othKey[1])
            && (key[2] == othKey[2]);
    }

    bool leaf;

    // Depth in the tree
    int depth;

    // Pivot dimension and value
    int pivot_dim;
    double pivot_value;

    // The key for this node
    int key[3];

    // The value for this node
    double value;

    // The cluster label (leaf nodes)
    int cluster;

    // Child nodes
    std::array<KdTreeNode*, 2> children;

};

// A kd tree
class KdTree {
public:
    explicit KdTree(size_t max_size);

    void clear();

    ~KdTree();

    // Insert a pose into the tree
    void InsertPose(Pose pose, double weight);

    // Insert a node into the tree
    KdTreeNode* InsertNode(KdTreeNode* parent, KdTreeNode* node, int key[], double value);


    KdTreeNode* FindNode(KdTreeNode* node, int key[]);

    // Cluster the leaves in the tree
    void Cluster();


    // Recursively label nodes in this cluster
    void ClusterNode(KdTreeNode* node, int depth);


    // Determine the cluster label for the given pose
    int GetCluster(Pose p);

    // Print node
    void PrintNode(KdTreeNode* node);

    // Cell size
    double size_[3];

    // The root node of the tree
    KdTreeNode *root_;
    std::vector<KdTreeNode*> nodes_;

    // The number of nodes in the tree
    size_t node_count_, node_max_count_;

    // The number of leaf nodes in the tree
    int leaf_count_;
};

}