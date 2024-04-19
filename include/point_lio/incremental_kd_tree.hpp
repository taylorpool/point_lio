#ifndef POINT_LIO_INCREMENTAL_KDTREE_HPP
#define POINT_LIO_INCREMENTAL_KDTREE_HPP

#include <Eigen/Dense>
#include <queue>
#include <vector>
#include <cmath>
#include <limits>
#include <functional>
#include <deque>

namespace point_lio {
    const int k = 3;
const float alpha = 0.5f;

struct Node
{
    Eigen::Vector3d point;
    Node *left, *right;
};

struct CompareDist {
    bool operator()(std::pair<double, Eigen::Vector3d> const &a, std::pair<double, Eigen::Vector3d> const &b) {
        return a.first < b.first;
    }
};

class KDTree {
public:

    KDTree();

    Node* root;
    std::deque<std::pair<Eigen::Vector3d, bool>> operationLogger;

    Node* newNode(int arr[]);
    Node* insertRec(Node* root, int point[], unsigned depth);
    Node* insert(int point[]);
    int treeSize(Node* root);
    int treeLeft(Node* root);
    int treeRight(Node* root);
    Node* build(const std::vector<Eigen::Vector3d>& v);
    void incrementalUpdates(Node* root, const std::pair<Eigen::Vector3d, bool>& operation, bool updateLogger);
    Node* removeNode(Node* root, Eigen::Vector3d point, unsigned depth);
    void parRebuild(Node *root);
    void reBalance(Node *root, unsigned depth);
    bool arePointsSame(const Eigen::Vector3d point1, const Eigen::Vector3d point2);
    bool searchRec(Node* root, Eigen::Vector3d point, unsigned depth);
    bool search(Node* root, Eigen::Vector3d point);
    void findNearestNeighbors(Node* root, const Eigen::Vector3d& target, std::priority_queue<std::pair<double, Eigen::Vector3d>, std::vector<std::pair<double, Eigen::Vector3d>>, CompareDist>& pq, int depth, int k);

};
}

#endif
