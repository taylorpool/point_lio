#include "point_lio/incremental_kd_tree.hpp"

namespace point_lio {

struct Node* newNode(int arr[])
{
    struct Node* temp = new Node;

    for (int i = 0; i < k; i++)
        temp->point[i] = arr[i];

    temp->left = temp->right = NULL;
    return temp;
}

Node *insertRec(Node *root, int point[], unsigned depth)
{
    if (root == NULL)
        return newNode(point);

    unsigned cd = depth % k;
    if (point[cd] < (root->point[cd]))
        root->left = insertRec(root->left, point, depth + 1);
    else
        root->right = insertRec(root->right, point, depth + 1);

    return root;
}

Node* insert(Node *root, int point[])
{
    return insertRec(root, point, 0);
}

int treeSize(Node* root) {
    if (root == nullptr) {
        return 0;
    }
    return 1 + treeSize(root->left) + treeSize(root->right);
}

int treeLeft(Node* root) {
    if (root == nullptr) {
        return 0;
    }
    return treeSize(root->left);
}

int treeRight(Node* root) {
    if (root == nullptr) {
        return 0;
    }
    return treeSize(root->right);
}

Node* build(const std::vector<Eigen::Vector3d>& v) {
    Node* root = nullptr;
    for (auto& point : v) {
        int arr[k];
        for (int i = 0; i < k; i++) {
            arr[i] = point[i];
        }
        root = insert(root, arr);
    }
    return root;
}

void incrementalUpdates(Node* root, const std::pair<Eigen::Vector3d, bool>& operation, bool updateLogger) {
    Eigen::Vector3d point = operation.first;
    bool insertOperation = operation.second;

    if (insertOperation) {
        int arr[k];
        for (int i = 0; i < k; i++) {
            arr[i] = point[i];
        }
        root = insert(root, arr);
    } else {
        if (searchRec(root, point, 0)) {
            root = removeNode(root, point, 0);
        }
    }
    if (updateLogger) {
        operationLogger.push_back(operation);
    }
    if (updateLogger) {
        operationLogger.push_back(operation);
    }
}

Node* removeNode(Node* root, Eigen::Vector3d point, unsigned depth) {
    if (root == nullptr)
        return nullptr;

    unsigned cd = depth % k;

    if (arePointsSame(root->point, point)) {
        if (root->left == nullptr && root->right == nullptr) {
            delete root;
            return nullptr;
        }
        if (root->left == nullptr) {
            Node* temp = root->right;
            delete root;
            return temp;
        }
        if (root->right == nullptr) {
            Node* temp = root->left;
            delete root;
            return temp;
        }
        Node* temp = root->right;
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

void parRebuild(Node *root) {
    lockUpdates(root);
    std::vector<Eigen::Vector3d> v = flatten(root);
    unlock(root);
    Node* newRoot = build(v);
    for (const auto& op : operationLogger) {
        incrementalUpdates(newRoot, op, false);
    }
    Node* temp = root;
    lockUpdates(root);
    root = newRoot;
    unlock(root);
}

void reBalance(Node *root, unsigned depth)
{
    int tree_size = treeSize(root);
    int tree_left = treeLeft(root);
    int tree_right = treeRight(root);

    if (tree_left < alpha * tree_size && tree_right < alpha * tree_size) {
    }
    else
    {
        parRebuild(root);
    }
}

bool arePointsSame(const Eigen::Vector3d point1, const Eigen::Vector3d point2)
{
    return (point1 - point2).squaredNorm() < 1e-3;
}

bool searchRec(Node* root, Eigen::Vector3d point, unsigned depth)
{
    if (root == NULL)
        return false;
    if (arePointsSame(root-> point, point))
        return true;

    unsigned cd = depth % k;

    if (point[cd] < root->point[cd])
        return searchRec(root->left, point, depth + 1);

    return searchRec(root->right, point, depth + 1);
}

bool search(Node* root, Eigen::Vector3d point)
{
    return searchRec(root, point, 0);
}


void findNearestNeighbors(Node* root, const Eigen::Vector3d& target, std::priority_queue<std::pair<double, Eigen::Vector3d>, std::vector<std::pair<double, Eigen::Vector3d>>, CompareDist>& pq, int depth = 0, int k = 5) {
    if (root == nullptr) return;

    unsigned int cd = depth % k;
    double dist = (root->point - target).norm();
    if (pq.size() < k) {
        pq.push(std::make_pair(dist, root->point));
    } else if (dist < pq.top().first) {
        pq.pop();
        pq.push(std::make_pair(dist, root->point));
    }
    Node* nearer = (target[cd] < root->point[cd]) ? root->left : root->right;
    Node* farther = (target[cd] < root->point[cd]) ? root->right : root->left;

    findNearestNeighbors(nearer, target, pq, depth + 1, k);
    if (farther != nullptr && (fabs(target[cd] - root->point[cd]) < pq.top().first)) {
        findNearestNeighbors(farther, target, pq, depth + 1, k);
    }
}

}
