#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>
#include <vector>

const int k = 3;
const float alpha = 0.5f;
std::deque<std::pair<Eigen::Vector3d, bool>> operationLogger;
std::mutex treeMutex;

struct Node {
  Eigen::Vector3d point;
  Node *left, *right;
};

struct CompareDist {
  bool operator()(std::pair<double, Eigen::Vector3d> const &a,
                  std::pair<double, Eigen::Vector3d> const &b) {
    return a.first < b.first;
  }
};

Node *newNode(const Eigen::Vector3d &point) {
  Node *temp = new Node;
  temp->point = point;
  temp->left = temp->right = NULL;
  return temp;
}

Node *insertRec(Node *root, const Eigen::Vector3d &point, unsigned depth) {
  if (root == NULL)
    return newNode(point);

  unsigned cd = depth % k;
  if (point[cd] < (root->point[cd]))
    root->left = insertRec(root->left, point, depth + 1);
  else
    root->right = insertRec(root->right, point, depth + 1);

  return root;
}

Node *insert(Node *root, const Eigen::Vector3d &point) {
  return insertRec(root, point, 0);
}

int treeSize(Node *root) {
  if (root == nullptr) {
    return 0;
  }
  return 1 + treeSize(root->left) + treeSize(root->right);
}

int treeLeft(Node *root) {
  if (root == nullptr) {
    return 0;
  }
  return treeSize(root->left);
}

int treeRight(Node *root) {
  if (root == nullptr) {
    return 0;
  }
  return treeSize(root->right);
}

Node *build(const std::vector<Eigen::Vector3d> &v) {
  Node *root = nullptr;
  for (const auto &point : v) {
    root = insert(root, point);
  }
  return root;
}

void incrementalUpdates(Node *&root,
                        const std::pair<Eigen::Vector3d, bool> &operation,
                        bool updateLogger) {
  Eigen::Vector3d point = operation.first;
  bool insertOperation = operation.second;

  if (insertOperation) {
    root = insert(root, point);
  } else {
    if (searchRec(root, point, 0)) {
      root = removeNode(root, point, 0);
    }
  }
  if (updateLogger) {
    operationLogger.push_back(operation);
  }
}

Node *removeNode(Node *root, const Eigen::Vector3d &point, unsigned depth) {
  if (root == nullptr)
    return nullptr;

  unsigned cd = depth % k;

  if (arePointsSame(root->point, point)) {
    if (root->left == nullptr && root->right == nullptr) {
      delete root;
      return nullptr;
    }
    if (root->left == nullptr) {
      Node *temp = root->right;
      delete root;
      return temp;
    }
    if (root->right == nullptr) {
      Node *temp = root->left;
      delete root;
      return temp;
    }
    Node *temp = root->right;
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

void lockUpdates() { treeMutex.lock(); }

void unlockUpdates() { treeMutex.unlock(); }

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

void parRebuild(Node *&root) {
  lockUpdates();
  std::vector<Eigen::Vector3d> v = flatten(root);
  unlockUpdates();
  Node *newRoot = build(v);
  for (const auto &op : operationLogger) {
    incrementalUpdates(newRoot, op, false);
  }
  Node *temp = root;
  lockUpdates();
  root = newRoot;
  unlockUpdates();
  delete temp;
}

void reBalance(Node *&root, unsigned depth) {
  int tree_size = treeSize(root);
  int tree_left = treeLeft(root);
  int tree_right = treeRight(root);

  if (tree_left < alpha * tree_size && tree_right < alpha * tree_size) {
    // No need for rebalancing
  } else {
    parRebuild(root);
  }
}

bool arePointsSame(const Eigen::Vector3d &point1,
                   const Eigen::Vector3d &point2) {
  return (point1 - point2).squaredNorm() < 1e-3;
}

bool searchRec(Node *root, const Eigen::Vector3d &point, unsigned depth) {
  if (root == NULL)
    return false;
  if (arePointsSame(root->point, point))
    return true;

  unsigned cd = depth % k;

  if (point[cd] < root->point[cd])
    return searchRec(root->left, point, depth + 1);

  return searchRec(root->right, point, depth + 1);
}

bool search(Node *root, const Eigen::Vector3d &point) {
  return searchRec(root, point, 0);
}

Eigen::MatrixXd findNearestNeighbors(const Eigen::Vector3d &target,
                                     const Eigen::MatrixXd &points,
                                     int near = 5) {
  std::priority_queue<std::pair<double, Eigen::Vector3d>,
                      std::vector<std::pair<double, Eigen::Vector3d>>,
                      CompareDist>
      pq;
  for (int i = 0; i < points.rows(); ++i) {
    const Eigen::Vector3d &point = points.row(i);
    double dist = (point - target).norm();
    if (pq.size() < near) {
      pq.push(std::make_pair(dist, point));
    } else if (dist < pq.top().first) {
      pq.pop();
      pq.push(std::make_pair(dist, point));
    }
  }

  Eigen::MatrixXd neighbors(near, 3);
  for (int i = 0; i < near; ++i) {
    neighbors.row(i) = pq.top().second;
    pq.pop();
  }

  return neighbors;
}

int main() {
  struct Node *root = NULL;
  Eigen::Vector3d point1(10, 19, 25);
  root = insert(root, point1);

  Eigen::Vector3d point2(12, 19, 30);
  root = insert(root, point2);

  // Eigen::Vector3d point_to_search(10, 19, 25);
  // (search(root, point_to_search)) ? std::cout << "Found\n" : std::cout <<
  // "Not Found\n";

  // // Check if rebalancing is needed and perform it if necessary
  // reBalance(root, 0);

  return 0;
}
