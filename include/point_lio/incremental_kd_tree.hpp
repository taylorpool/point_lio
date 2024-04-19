#ifndef POINT_LIO_INCREMENTAL_KDTREE_HPP
#define POINT_LIO_INCREMENTAL_KDTREE_HPP

#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <vector>

namespace point_lio {
const int k = 3;
const float alpha = 0.5f;

struct Node {
  using Ptr = Node *;
  Eigen::Vector3d point;
  Ptr left;
  Ptr right;

  Node(Eigen::Vector3d point);
};

struct CompareDist {
  bool operator()(std::pair<double, Eigen::Vector3d> const &a,
                  std::pair<double, Eigen::Vector3d> const &b) {
    return a.first < b.first;
  }
};

Node::Ptr newNode(Eigen::Vector3d point);

Node::Ptr insert(Eigen::Vector3d point);

Node::Ptr insertRec(Node::Ptr root, Eigen::Vector3d point, unsigned depth);

int treeSize(Node::Ptr root);

int treeLeft(Node::Ptr root);

int treeRight(Node::Ptr root);

Node::Ptr build(const std::vector<Eigen::Vector3d> &v);

bool arePointsSame(const Eigen::Vector3d point1, const Eigen::Vector3d point2);

class KDTree {
public:
  KDTree();

  Node::Ptr root;

  std::deque<std::pair<Eigen::Vector3d, bool>> operationLogger;

  void incrementalUpdates(Node::Ptr root,
                          const std::pair<Eigen::Vector3d, bool> &operation,
                          bool updateLogger);

  Node::Ptr removeNode(Node::Ptr root, Eigen::Vector3d point, unsigned depth);

  void parRebuild(Node::Ptr root);

  void reBalance(Node::Ptr root, unsigned depth);

  bool searchRec(Node::Ptr root, Eigen::Vector3d point, unsigned depth);

  bool search(Node::Ptr root, Eigen::Vector3d point);

  void findNearestNeighbors(
      Node::Ptr root, const Eigen::Vector3d &target,
      std::priority_queue<std::pair<double, Eigen::Vector3d>,
                          std::vector<std::pair<double, Eigen::Vector3d>>,
                          CompareDist> &pq,
      int depth, int k);
};
} // namespace point_lio

#endif
