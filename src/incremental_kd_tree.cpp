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
  return treeSize(root->right);
}

Node::Ptr insert(Node::Ptr root, const Eigen::Vector3d& point, unsigned depth = 0) {
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

Node::Ptr build(const std::vector<Eigen::Vector3d>& v, KDTree& kdtree) {
    Node::Ptr root = nullptr;

    for (const auto& point : v) {
        std::pair<Eigen::Vector3d, bool> operation(point, true);
        kdtree.incrementalUpdates(root, operation, false); 
    }
    return root;
}
void KDTree::incrementalUpdates(Node::Ptr& root, const std::pair<Eigen::Vector3d, bool>& operation, bool updateLogger) {
    const Eigen::Vector3d& point = operation.first;
    bool isInsertOperation = operation.second;
    if (isInsertOperation) {
        root = insert(root, point);
    } else {
        if (search(root, point)) {
            root = removeNode(root, point, 0);
        }
    }
}

void lockUpdates() { treeMutex.lock(); }
void unlockUpdates() { treeMutex.unlock(); }

void parRebuild(Node::Ptr root) {
    lockUpdates();
    std::vector<Eigen::Vector3d> v = flatten(root);
    unlockUpdates();

    KDTree kdtree;
    Node::Ptr newRoot = build(v, kdtree);

    lockUpdates();
    root = newRoot;
    unlockUpdates();
}

void reBalance(Node::Ptr root, unsigned depth) {
    int tree_size = treeSize(root);
    int tree_left = treeLeft(root);
    int tree_right = treeRight(root);

    if (tree_left < alpha * tree_size && tree_right < alpha * tree_size) {
    } else {
    
    }
}

Node::Ptr removeNode(Node::Ptr root, Eigen::Vector3d point, unsigned depth) {
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
}