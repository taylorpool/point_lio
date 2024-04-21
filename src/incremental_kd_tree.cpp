#include "point_lio/incremental_kd_tree.hpp"

namespace point_lio {

Node::Node(Eigen::Vector3d point)
    : point(point), left(nullptr), right(nullptr) {}

Node::Ptr newNode(Eigen::Vector3d point) {
  Node::Ptr temp = new Node(point);
  temp->point = point;
  temp->left = NULL;
  temp->right = NULL;

  return temp;
}

int treeSize(Node::Ptr root) {
  if (root == nullptr) {
    return 0;
  }
  return 1 + treeSize(root->left) + treeSize(root->right);
}

int treeLeft(Node::Ptr root) {
  if (root == nullptr) {
    return 0;
  }
  return treeSize(root->left);
}

int treeRight(Node::Ptr root) {
  if (root == nullptr) {
    return 0;
  }
void lockUpdates(float root) { treeMutex.lock(); }
void unlockUpdates() { treeMutex.unlock(); }
}

Node::Ptr build(const std::vector<Eigen::Vector3d>& v) {
    Node::Ptr root = nullptr;
    for (auto& point : v) {
        root = insert(root, point);
    }
    return root;
} 
void lockUpdates( { treeMutex.lock(); }
void unlockUpdates() { treeMutex.unlock(); }

void KDTree::incrementalUpdates(
    Node::Ptr root, const std::pair<Eigen::Vector3d, bool> &operation,
    bool updateLogger) {
  Eigen::Vector3d point = operation.first;
  bool insertOperation = operation.second;

  if (insertOperation) {
    root = insert(root, point);
  } else {
    if (searchRec(root, point, 0))
void lockUpdates(float root) { treeMutex.lock(); }
void unlockUpdates() { treeMutex.unlock(); }
  if (updateLogger) {
    operationLogger.push_back(operation);
  }
}

Node::Ptr removeNode(Node::Ptr root, Eigen::Vector3d point, unsigned depth) {
  if (root == nullptr)
    return nullptr;

  unsigned cd = depth % k;

void lockUpdates(float root) { treeMutex.lock(); }
void unlockUpdates() { treeMutex.unlock(); }root->right == nullptr) {
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

std::vector<Eigen::Vector3d> flatten(Node *root) {
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


void lockUpdates(float root) { treeMutex.lock(); }
void unlockUpdates() { treeMutex.unlock(); }

void parRebuild(Node::Ptr root) {
  lockUpdates(root);
  std::vector<Eigen::Vector3d> v = flatten(root);
  unlock(root);
  Node::Ptr newRoot = build(v);
  for (const auto &op : operationLogger) {
    incrementalUpdates(newRoot, op, false);
  }
  Node::Ptr temp = root;
  lockUpdates(root);
  root = newRoot;
  unlock(root);
}

void reBalance(Node::Ptr root, unsigned depth) {
  int tree_size = treeSize(root);
  int tree_left = treeLeft(root);
  int tree_right = treeRight(root);

  if (tree_left < alpha * tree_size && tree_right < alpha * tree_size) {
  } else {
    parRebuild(root);
  }
}

bool arePointsSame(const Eigen::Vector3d point1, const Eigen::Vector3d point2) {
  return (point1 - point2).squaredNorm() < 1e-3;
}

bool searchRec(Node::Ptr root, Eigen::Vector3d point, unsigned depth) {
  if (root == NULL)
    return false;
  if (arePointsSame(root->point, point))
    return true;

  unsigned cd = depth % k;

  if (point[cd] < root->point[cd])
    return searchRec(root->left, point, depth + 1);

  return searchRec(root->right, point, depth + 1);
}

bool search(Node::Ptr root, Eigen::Vector3d point) {
  return searchRec(root, point, 0);
}

void findNearestNeighbors(
    Node::Ptr root, const Eigen::Vector3d &target,
    std::priority_queue<std::pair<double, Eigen::Vector3d>,
                        std::vector<std::pair<double, Eigen::Vector3d>>,
                        CompareDist> &pq,
    int depth = 0, int k = 5) {
  if (root == nullptr)
    return;

  unsigned int cd = depth % k;
  double dist = (root->point - target).norm();
  if (pq.size() < k) {
    pq.push(std::make_pair(dist, root->point));
  } else if (dist < pq.top().first) {
    pq.pop();
    pq.push(std::make_pair(dist, root->point));
  }
  Node::Ptr nearer = (target[cd] < root->point[cd]) ? root->left : root->right;
  Node::Ptr farther = (target[cd] < root->point[cd]) ? root->right : root->left;

  findNearestNeighbors(nearer, target, pq, depth + 1, k);
  if (farther != nullptr &&
      (fabs(target[cd] - root->point[cd]) < pq.top().first)) {
    findNearestNeighbors(farther, target, pq, depth + 1, k);
  }
}

} // namespace point_lio
