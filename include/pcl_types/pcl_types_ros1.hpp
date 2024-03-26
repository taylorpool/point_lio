#ifndef PCL_TYPES_PCL_TYPES_ROS1_HPP
#define PCL_TYPES_PCL_TYPES_ROS1_HPP

#include "pcl_types/pcl_types.hpp"

#include <sensor_msgs/PointCloud2.h>

#include <concepts>

namespace pcl_types::ros1 {

template <typename T>
  requires std::is_same_v<T, float32_t>
[[nodiscard]] consteval auto getPointField() noexcept {
  return sensor_msgs::PointField::FLOAT32;
}

template <typename T>
  requires std::is_same_v<T, float64_t>
[[nodiscard]] consteval auto getPointField() noexcept {
  return sensor_msgs::PointField::FLOAT64;
}

template <typename T>
  requires std::is_same_v<T, uint8_t>
[[nodiscard]] consteval auto getPointField() noexcept {
  return sensor_msgs::PointField::UINT8;
}

template <typename T>
  requires std::is_same_v<T, uint16_t>
[[nodiscard]] consteval auto getPointField() noexcept {
  return sensor_msgs::PointField::UINT16;
}

template <typename T>
  requires std::is_same_v<T, uint32_t>
[[nodiscard]] consteval auto getPointField() noexcept {
  return sensor_msgs::PointField::UINT32;
}

template <typename T>
  requires std::is_same_v<T, int8_t>
[[nodiscard]] consteval auto getPointField() noexcept {
  return sensor_msgs::PointField::INT8;
}

template <typename T>
  requires std::is_same_v<T, int16_t>
[[nodiscard]] consteval auto getPointField() noexcept {
  return sensor_msgs::PointField::INT16;
}

template <typename T>
  requires std::is_same_v<T, int32_t>
[[nodiscard]] consteval auto getPointField() noexcept {
  return sensor_msgs::PointField::INT32;
}

[[nodiscard]] bool fromMsg(const sensor_msgs::PointCloud2 &msg,
                           pcl_types::LidarScanStamped &scan) noexcept;

[[nodiscard]] bool toMsg(const std::vector<pcl_types::PointXYZICT> &points,
                         sensor_msgs::PointCloud2 &msg) noexcept;

[[nodiscard]] bool toMsg(const pcl_types::LidarScanStamped &scan,
                         sensor_msgs::PointCloud2 &msg) noexcept;

} // namespace pcl_types::ros1

#endif
