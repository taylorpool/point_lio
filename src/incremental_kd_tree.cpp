#include "point_lio/incremental_kd_tree.hpp"

namespace point_lio {

Node::Node(Eigen::Vector3d point)
    : point(point), left(nullptr), right(nullptr) {}

Node::Ptr KDTree::newNode(Eigen::Vector3d point) {
  Node::Ptr temp = new Node(point);
  temp->point = point;
  temp->left = NULL;
  temp->right = NULL;

  return temp;
}

int KDTree::treeSize(Node::Ptr root) {
  if (root == nullptr) {
    return 0;
  }
  return 1 + treeSize(root->left) + treeSize(root->right);
}

int KDTree::treeLeft(Node::Ptr root) {
  if (root == nullptr) {
    return 0;
  }
  return treeSize(root->left);
}

int KDTree::treeRight(Node::Ptr root) {
  if (root == nullptr) {
    return 0;
  }
  return treeSize(root->right);
}

Node::Ptr KDTree::insert(Node::Ptr root, const Eigen::Vector3d &point,
                         unsigned depth = 0) {
  if (root == nullptr) {
    return newNode(point);
  }
  unsigned cd = depth % k;
  if (point[cd] < root->point[cd]) {
    root->left = insert(root->left, point, depth + 1);
  } else {
    root->right = insert(root->right, point, depth + 1);
  }
  return root;
}

Node::Ptr KDTree::build2(const Eigen::Vector3d point) {
  Node::Ptr root = nullptr;

  std::pair<Eigen::Vector3d, bool> operation(point, true);
  incrementalUpdates(root, operation, false);
  return root;
}

Node::Ptr KDTree::build(const std::vector<Eigen::Vector3d> &v) {
  Node::Ptr root = nullptr;

  for (const auto &point : v) {
    std::pair<Eigen::Vector3d, bool> operation(point, true);
    incrementalUpdates(root, operation, false);
  }
  return root;
}

void KDTree::incrementalUpdates(
    Node::Ptr &root, const std::pair<Eigen::Vector3d, bool> &operation,
    bool updateLogger) {
  const Eigen::Vector3d &point = operation.first;
  bool isInsertOperation = operation.second;
  if (isInsertOperation) {
    root = insert(root, point);
  } else {
    if (search(root, point)) {
      root = removeNode(root, point, 0);
    }
  }
}

void KDTree::lockUpdates() { treeMutex.lock(); }
void KDTree::unlockUpdates() { treeMutex.unlock(); }

void KDTree::parRebuild(Node::Ptr &root) {
  lockUpdates();
  std::vector<Eigen::Vector3d> v = flatten(root);
  unlockUpdates();

  // KDTree kdtree;
  Node::Ptr newRoot = build(v);

  lockUpdates();
  root = newRoot;
  unlockUpdates();
}

void KDTree::reBalance(Node::Ptr root, unsigned depth) {
  int tree_size = treeSize(root);
  int tree_left = treeLeft(root);
  int tree_right = treeRight(root);

  if (tree_left < alpha * tree_size && tree_right < alpha * tree_size) {
  } else {
  }
}

Node::Ptr KDTree::removeNode(Node::Ptr root, Eigen::Vector3d point,
                             unsigned depth) {
  if (root == nullptr)
    return nullptr;

  unsigned cd = depth % k;

  if (arePointsSame(root->point, point)) {
    if (root->left == nullptr && root->right == nullptr) {
      delete root;
      return nullptr;
    }
    if (root->left == nullptr) {
      Node::Ptr temp = root->right;
      delete root;
      return temp;
    }
    if (root->right == nullptr) {
      Node::Ptr temp = root->left;
      delete root;
      return temp;
    }

    Node::Ptr temp = root->right;
    while (temp->left != nullptr)
      temp = temp->left;
    root->point = temp->point;
    root->right = removeNode(root->right, temp->point, depth + 1);
  } else if (point[cd] < root->point[cd]) {
    root->left = removeNode(root->left, point, depth + 1);
  } else {
    root->right = removeNode(root->right, point, depth + 1);
  }

  return root;
}

std::vector<Eigen::Vector3d> KDTree::flatten(Node *root) {
  std::vector<Eigen::Vector3d> points;
  std::function<void(Node *)> traverse = [&](Node *node) {
    if (node == nullptr) {
      return;
    }
    points.push_back(node->point);
    traverse(node->left);
    traverse(node->right);
  };
  traverse(root);
  return points;
}

bool KDTree::searchRec(Node::Ptr root, Eigen::Vector3d point, unsigned depth) {
  if (root == NULL)
    return false;
  if (arePointsSame(root->point, point))
    return true;

  unsigned cd = depth % k;

  if (point[cd] < root->point[cd])
    return searchRec(root->left, point, depth + 1);

  return searchRec(root->right, point, depth + 1);
}

bool KDTree::search(Node::Ptr root, Eigen::Vector3d point) {
  return searchRec(root, point, 0);
}

bool arePointsSame(const Eigen::Vector3d point1, const Eigen::Vector3d point2) {
  return (point1 - point2).squaredNorm() < 1e-3;
}

KDTree::KDTree() : root(nullptr) {}

Eigen::Matrix<double, 5, 3>
KDTree::findNearestNeighbors(const Eigen::Vector3d &target) {
  int numNearest = 5;
  std::lock_guard<std::mutex> guard(treeMutex);
  std::priority_queue<std::pair<double, Eigen::Vector3d>,
                      std::vector<std::pair<double, Eigen::Vector3d>>,
                      CompareDist>
      pq;

  std::function<void(Node::Ptr, unsigned)> traverse = [&](Node::Ptr node,
                                                          unsigned depth) {
    if (!node)
      return;

    unsigned cd = depth % k;
    double dist = (node->point - target).norm();
    if (pq.size() < numNearest) {
      pq.push({dist, node->point});
    } else if (dist < pq.top().first) {
      pq.pop();
      pq.push({dist, node->point});
    }
    double diff = target[cd] - node->point[cd];
    Node::Ptr first = (diff < 0) ? node->left : node->right;
    Node::Ptr second = (diff < 0) ? node->right : node->left;

    traverse(first, depth + 1);
    if (fabs(diff) < pq.top().first) {
      traverse(second, depth + 1);
    }
  };

  traverse(root, 0);

  Eigen::Matrix<double, 5, 3> neighbors;
  int i = pq.size() - 1;
  while (!pq.empty()) {
    neighbors.row(i--) = pq.top().second;
    pq.pop();
  }
  return neighbors;
}

// Eigen::MatrixXd KDTree::findNearestNeighbors(const Eigen::Vector3d& target,
// const Eigen::MatrixXd& points, int near = 5) {
//    std::priority_queue<std::pair<double, Eigen::Vector3d>,
//    std::vector<std::pair<double, Eigen::Vector3d>>, CompareDist> pq; for (int
//    i = 0; i < points.rows(); ++i) {
//        const Eigen::Vector3d& point = points.row(i);
//        double dist = (point - target).norm();
//        if (pq.size() < near) {
//            pq.push(std::make_pair(dist, point));
//        } else if (dist < pq.top().first) {
//            pq.pop();
//            pq.push(std::make_pair(dist, point));
//        }
//    }

//    Eigen::MatrixXd neighbors(near, 3);
//    for (int i = 0; i < near; ++i) {
//        neighbors.row(i) = pq.top().second;
//        pq.pop();
//    }

//    return neighbors;
// }
} // namespace point_lio
