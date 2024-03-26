#include "point_lio_ros1/point_lio_ros1.hpp"

namespace point_lio::ros1 {

[[nodiscard]] bool fromMsg(const ros::Time &msg,
                           point_lio::Stamp &stamp) noexcept {
  stamp = msg.toSec();
  return true;
}

[[nodiscard]] bool fromMsg(const geometry_msgs::Vector3 msg,
                           Eigen::Vector3d &vec) noexcept {
  vec.x() = msg.x;
  vec.y() = msg.y;
  vec.z() = msg.z;
  return true;
}

[[nodiscard]] bool fromMsg(const sensor_msgs::Imu &msg,
                           point_lio::Imu &imu) noexcept {
  return fromMsg(msg.header.stamp, imu.stamp) &&
         fromMsg(msg.angular_velocity, imu.body_measuredAngularVelocity) &&
         fromMsg(msg.linear_acceleration, imu.body_measuredLinearAcceleration);
}
} // namespace point_lio::ros1
