// #ifndef POINT_LIO_INCREMENTAL_KDTREE_HPP
// #define POINT_LIO_INCREMENTAL_KDTREE_HPP

// #include <Eigen/Dense>
// #include <cmath>
// #include <deque>
// #include <functional>
// #include <limits>
// #include <memory>
// #include <mutex>
// #include <queue>
// #include <vector>

// namespace point_lio {

// const int k = 3;
// const float alpha = 0.5f;

// struct Node {
//     using Ptr = Node*;
//     Eigen::Vector3d point;
//     Ptr left;
//     Ptr right;
//     Ptr father;

//     Node(Eigen::Vector3d point); // Only declaration here
// };

// struct CompareDist {
//     bool operator()(std::pair<double, Eigen::Vector3d> const &a,
//                     std::pair<double, Eigen::Vector3d> const &b) {
//         return a.first < b.first;
//     }
// };

// bool arePointsSame(const Eigen::Vector3d point1, const Eigen::Vector3d point2);

// class KDTree {
// public:
//     std::mutex treeMutex;
//     Node::Ptr root;

//     KDTree(); // Constructor declared if needed

//     void incrementalUpdates(Node::Ptr &root,
//                             const std::pair<Eigen::Vector3d, bool> &operation,
//                             bool updateLogger);
//     Node::Ptr removeNode(Node::Ptr root, Eigen::Vector3d point, unsigned depth);
//     void parRebuild(Node::Ptr &root);
//     void reBalance(Node::Ptr root, unsigned depth);
//     bool searchRec(Node::Ptr root, Eigen::Vector3d point, unsigned depth);
//     bool search(Node::Ptr root, Eigen::Vector3d point);
//     Eigen::Matrix<double, 5, 3> findNearestNeighbors(const Eigen::Vector3d &target);
//     void lockUpdates();
//     Eigen::Vector3d KDTree::FindCube(double l, const Eigen::Vector3d& point);
//     void unlockUpdates();
//     void downsampleAndInsert(double l, const Eigen::Vector3d& p, bool SW);
//     Node::Ptr newNode(Eigen::Vector3d point);
//     Node::Ptr insertRec(Node::Ptr root, Eigen::Vector3d point, unsigned depth);
//     int treeSize(Node::Ptr root);
//     int treeLeft(Node::Ptr root);
//     int treeRight(Node::Ptr root);
//     Node::Ptr build2(const Eigen::Vector3d point);
//     std::vector<Eigen::Vector3d> flatten(Node *root);
//     void insert(Node::Ptr& root, const Eigen::Vector3d& point, Node::Ptr father, bool SW);
//     Node::Ptr insert(Node::Ptr root, const Eigen::Vector3d &point, unsigned depth);
//     Node::Ptr build(const std::vector<Eigen::Vector3d> &v);
// };

// } // namespace point_lio

// #endif // POINT_LIO_INCREMENTAL_KDTREE_HPP
