#ifndef UTILS_ROS1_UTILS_ROS1_HPP
#define UTILS_ROS1_UTILS_ROS1_HPP

#include <Eigen/Dense>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include <ros/ros.h>

#include <cstdint>
#include <string>

namespace utils::ros1 {

struct TopicParams {
  std::string topic;
  uint32_t queueSize;
};

template <typename T>
concept RosParamT =
    std::is_same_v<T, float> || std::is_same_v<T, double> ||
    std::is_same_v<T, std::string> || std::is_same_v<T, bool> ||
    std::is_same_v<T, int> || std::is_same_v<T, std::vector<bool>> ||
    std::is_same_v<T, std::vector<float>> ||
    std::is_same_v<T, std::vector<double>>;

template <RosParamT T>
[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            T &param) noexcept {
  if (!node.getParam(paramName, param)) {
    std::cerr << "Error: Could not find param\n";
    return false;
  } else {
    return true;
  }
}

template <RosParamT T>
[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            T &param, const T backup) noexcept {
  if (!node.getParam(paramName, param)) {
    param = backup;
  }
  return true;
}

[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            size_t &param) noexcept;
[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            uint32_t &param) noexcept;
[[nodiscard]] bool getParam(ros::NodeHandle &node, TopicParams &param) noexcept;
[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            TopicParams &param) noexcept;
[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            Eigen::VectorXd &param) noexcept;
[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            Eigen::Vector3d &param) noexcept;
[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            Eigen::Matrix<double, 3, 3> &param) noexcept;
[[nodiscard]] bool getParam(ros::NodeHandle &node, const std::string &paramName,
                            Eigen::Matrix<double, 6, 6> &param) noexcept;

template <typename T>
  requires std::is_arithmetic_v<T>
[[nodiscard]] bool getParamAndSquare(ros::NodeHandle &node,
                                     const std::string &paramName,
                                     T &param) noexcept {
  if (getParam(node, paramName, param)) {
    param *= param;
    return true;
  } else {
    return false;
  }
}

void toMsg(const double stamp, ros::Time &time) noexcept;
void toMsg(const Eigen::Vector3d vector, geometry_msgs::Vector3 &msg) noexcept;
void toMsg(const Eigen::Vector3d point, geometry_msgs::Point &msg) noexcept;

void fromMsg(const ros::Time &msg, double &stamp) noexcept;
void fromMsg(const geometry_msgs::Vector3 &msg, Eigen::Vector3d &vec) noexcept;
void fromMsg(const geometry_msgs::Point &msg, Eigen::Vector3d &vec) noexcept;

template <typename T>
[[nodiscard]] ros::Publisher advertise(ros::NodeHandle &node,
                                       const TopicParams &params) noexcept {
  return node.advertise<T>(params.topic, params.queueSize);
}

[[nodiscard]] ros::NodeHandle operator/(const ros::NodeHandle &node,
                                        const std::string &ns) noexcept;

} // namespace utils::ros1

#endif
