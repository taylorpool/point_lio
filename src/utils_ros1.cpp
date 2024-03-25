#include "utils_ros1/utils_ros1.hpp"

#include <format>
#include <iostream>

namespace utils::ros1 {

[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            size_t &param) noexcept {
  int temp;
  if (getParam(node, paramName, temp)) {
    param = static_cast<size_t>(temp);
    return true;
  } else {
    return false;
  }
}

[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            uint32_t &param) noexcept {
  int temp;
  if (getParam(node, paramName, temp)) {
    param = static_cast<uint32_t>(temp);
    return true;
  } else {
    return false;
  }
}

[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            Eigen::VectorXd &param) noexcept {
  std::vector<double> temp;
  if (getParam(node, paramName, temp)) {
    for (size_t i = 0; i < temp.size(); ++i) {
      param(i) = temp[i];
    }
    return true;
  } else {
    return false;
  }
}

[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            Eigen::Vector3d &param) noexcept {
  std::vector<double> temp;
  if (getParam(node, paramName, temp)) {
    param(0) = temp[0];
    param(1) = temp[1];
    param(2) = temp[2];
    return true;
  } else {
    return false;
  }
}

[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            Eigen::Matrix<double, 3, 3> &param) noexcept {
  std::vector<double> temp;
  if (getParam(node, paramName, temp)) {
    param(0, 0) = temp[0];
    param(0, 1) = temp[1];
    param(0, 2) = temp[2];
    param(1, 0) = temp[3];
    param(1, 1) = temp[4];
    param(1, 2) = temp[5];
    param(2, 0) = temp[6];
    param(2, 1) = temp[7];
    param(2, 2) = temp[8];
    return true;
  } else {
    return false;
  }
}

[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            Eigen::Matrix<double, 6, 6> &param) noexcept {
  std::vector<double> temp;
  if (getParam(node, paramName, temp)) {
    size_t index = 0;
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        param(row, col) = temp[index];
        ++index;
      }
    }
    return true;
  } else {
    return false;
  }
}

[[nodiscard]] bool getParam(ros::NodeHandle &node,
                            TopicParams &param) noexcept {
  return getParam(node, "topic", param.topic) &&
         getParam(node, "queue_size", param.queueSize);
}

[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            TopicParams &param) noexcept {
  auto paramNode = node / paramName;
  return getParam(paramNode, param);
}

void toMsg(const double stamp, ros::Time &time) noexcept {
  time.fromSec(stamp);
}

void toMsg(const Eigen::Vector3d vector, geometry_msgs::Vector3 &msg) noexcept {
  msg.x = vector.x();
  msg.y = vector.y();
  msg.z = vector.z();
}

void toMsg(const Eigen::Vector3d point, geometry_msgs::Point &msg) noexcept {
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
}

void fromMsg(const ros::Time &msg, double &stamp) noexcept {
  stamp = msg.toSec();
}

void fromMsg(const geometry_msgs::Vector3 &msg, Eigen::Vector3d &vec) noexcept {
  vec.x() = msg.x;
  vec.y() = msg.y;
  vec.z() = msg.z;
}

void fromMsg(const geometry_msgs::Point &msg, Eigen::Vector3d &vec) noexcept {
  vec.x() = msg.x;
  vec.y() = msg.y;
  vec.z() = msg.z;
}

[[nodiscard]] ros::NodeHandle operator/(const ros::NodeHandle &node,
                                        const std::string &ns) noexcept {
  return {node, ns};
}

} // namespace utils::ros1
