#include "point_lio/incremental_kd_tree.hpp"

namespace point_lio {

Node::Node(Eigen::Vector3d point)
    : point(point), left(nullptr), right(nullptr) {}

Node::Ptr newNode(Eigen::Vector3d point) {
  return std::make_shared<Node>(point);
}

int treeSize(Node::Ptr root) {
  return root ? 1 + treeSize(root->left) + treeSize(root->right) : 0;
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

void KDTree::insert(Node::Ptr& root, const Eigen::Vector3d& point, Node::Ptr father, bool SW) {
    if (!root) {
        root = newNode(point);
        root->father = father;
        return;
    }

    unsigned ax = depth++ % k;
    if (point[ax] < root->point[ax]) {
        insert(root->left, point, root, SW);
    } else {
        insert(root->right, point, root, SW);
    }

    AttributeUpdate(root);
    Rebalance(root, SW);
}

void KDTree::downsampleAndInsert(double l, const Eigen::Vector3d& p, bool SW) {
    Eigen::Vector3d CD = FindCube(l, p);
    Eigen::Vector3d pcenter = Center(CD);
    std::vector<Eigen::Vector3d> V = BoxwiseSearch(root, CD);
    V.push_back(p);
    Eigen::Vector3d pnearest = FindNearest(V, pcenter);
    BoxwiseDelete(root, CD);
    insert(root, pnearest, nullptr, SW);
}



std::vector<Eigen::Vector3d> KDTree::BoxwiseSearch(Node::Ptr root, const Eigen::Vector3d& CD) {
    std::vector<Eigen::Vector3d> points;
    std::function<void(Node::Ptr)> traverse = [&](Node::Ptr node) {
        if (!node) return;

        if ((node->point.array() >= CD.array()).all() && (node->point.array() <= (CD + CD).array()).all()) {
            points.push_back(node->point);
        }

        traverse(node->left);
        traverse(node->right);
    };
    traverse(root);
    return points;
}

Node::Ptr build(const std::vector<Eigen::Vector3d>& v, KDTree& kdtree) {
    Node::Ptr root = nullptr;
    for (const auto& point : v) {
        kdtree.insert(root, point, nullptr, false);
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

void parRebuild(Node::Ptr& root) {
    lockUpdates(); 
    std::vector<Eigen::Vector3d> points = flatten(root);
    unlockUpdates();

    KDTree kdtree;
    Node::Ptr newRoot = build(points, kdtree); 

    lockUpdates();
    root = newRoot; 
    unlockUpdates();
}

std::vector<Eigen::Vector3d> reBalance(Node::Ptr& root, unsigned depth = 0) {
    if (!root) {
        return std::vector<Eigen::Vector3d>(); 
    int tree_size = treeSize(root);
    int tree_left = treeLeft(root);
    int tree_right = treeRight(root);

    if (!(tree_left < alpha * tree_size && tree_right < alpha * tree_size)) {
        parRebuild(root);
    }
    std::vector<Eigen::Vector3d> points = flatten(root);
    return points;
}


Node::Ptr removeNode(Node::Ptr root, Eigen::Vector3d point, unsigned depth) {
  if (root == nullptr)
    return nullptr;

  unsigned cd = depth % k;

  if (arePointsSame(root->point, point)) {
    if (!root->left && !root->right) {
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
  if (!root || arePointsSame(root->point, point))
    return root != nullptr;

  unsigned cd = depth % k;

  if (point[cd] < root->point[cd])
    return searchRec(root->left, point, depth + 1);

  return searchRec(root->right, point, depth + 1);
}

  if (point[cd] < root->point[cd])
    return searchRec(root->left, point, depth + 1);

  return searchRec(root->right, point, depth + 1);
}

bool search(Node::Ptr root, Eigen::Vector3d point) {
  return searchRec(root, point, 0);
}

void boxwiseDelete(Node::Ptr& root, const Eigen::Vector3d& lowerBound, const Eigen::Vector3d& upperBound, unsigned depth = 0) {
    if (!root) return;

    unsigned cd = depth % k;

    if (root->point[cd] < lowerBound[cd]) {
        boxwiseDelete(root->right, lowerBound, upperBound, depth + 1);
    } else if (root->point[cd] > upperBound[cd]) {
        boxwiseDelete(root->left, lowerBound, upperBound, depth + 1);
    } else {
        boxwiseDelete(root->left, lowerBound, upperBound, depth + 1);
        boxwiseDelete(root->right, lowerBound, upperBound, depth + 1);
        if ((root->point.array() >= lowerBound.array()).all() && (root->point.array() <= upperBound.array()).all()) {
            root = removeNode(root, root->point, depth);
        }
    }
}

Eigen::Vector3d KDTree::FindNearest(const std::vector<Eigen::Vector3d>& V, const Eigen::Vector3d& pcenter) {
    double minDist = std::numeric_limits<double>::max();
    Eigen::Vector3d nearest;

    for (const auto& point : V) {
        double dist = (point - pcenter).norm();
        if (dist < minDist) {
            minDist = dist;
            nearest = point;
        }
    }

    return nearest;
}
void KDTree::AttributeUpdate(Node::Ptr& node) { /// NOTE ADD HEIGHT STUCT NODE
    int leftHeight = node->left ? node->left->height : -1;
    int rightHeight = node->right ? node->right->height : -1;
    node->height = 1 + std::max(leftHeight, rightHeight);

Node::Ptr KDTree::buildBalancedTree(const std::vector<Eigen::Vector3d>& points, unsigned depth) {
    if (points.empty()) return nullptr;

    unsigned axis = depth % k;
    std::vector<Eigen::Vector3d>::const_iterator median = points.begin() + points.size() / 2;
    std::nth_element(points.begin(), median, points.end(), [axis](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return a[axis] < b[axis];
    });

    Node::Ptr node = newNode(*median);
    std::vector<Eigen::Vector3d> leftPoints(points.begin(), median);
    std::vector<Eigen::Vector3d> rightPoints(median + 1, points.end());

    node->left = buildBalancedTree(leftPoints, depth + 1);
    node->right = buildBalancedTree(rightPoints, depth + 1);

    return node;
}
Eigen::Vector3d KDTree::FindCube(double l, const Eigen::Vector3d& point) {
    return point - Eigen::Vector3d(l/2, l/2, l/2);
}
Eigen::Vector3d KDTree::Center(const Eigen::Vector3d& lowerCorner) {
    double length = 2 * lowerCorner.maxCoeff(); 
    return lowerCorner + Eigen::Vector3d(length/2, length/2, length/2);
}

Eigen::MatrixXd findNearestNeighbors(const Eigen::Vector3d& target) {
    std::lock_guard<std::mutex> guard(treeMutex);
    std::priority_queue<std::pair<double, Eigen::Vector3d>, std::vector<std::pair<double, Eigen::Vector3d>>, CompareDist> pq;

    std::function<void(Node::Ptr, unsigned)> traverse = [&](Node::Ptr node, unsigned depth) {
        if (!node) return;

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

    Eigen::MatrixXd neighbors(pq.size(), 3);
    int i = pq.size() - 1;
    while (!pq.empty()) {
        neighbors.row(i--) = pq.top().second;
        pq.pop();
    }
    return neighbors;
}
}
};