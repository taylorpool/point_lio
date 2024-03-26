#ifndef POINT_LIO_POINT_LIO_ROS1_HPP
#define POINT_LIO_POINT_LIO_ROS1_HPP

#include "point_lio/point_lio.hpp"

#include <sensor_msgs/Imu.h>

namespace point_lio::ros1 {

[[nodiscard]] bool fromMsg(const ros::Time &msg,
                           point_lio::Stamp &stamp) noexcept;

[[nodiscard]] bool fromMsg(const geometry_msgs::Vector3,
                           Eigen::Vector3d &vec) noexcept;

[[nodiscard]] bool fromMsg(const sensor_msgs::Imu &msg,
                           point_lio::Imu &imu) noexcept;

} // namespace point_lio::ros1

#endif
