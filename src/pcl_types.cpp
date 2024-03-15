#include "pcl_types/pcl_types.hpp"

namespace pcl_types {

[[nodiscard]] Eigen::Map<Eigen::Vector3f> PointXYZICT::getVec3Map() noexcept {
  return Eigen::Map<Eigen::Vector3f>(&x);
}

[[nodiscard]] Eigen::Map<const Eigen::Vector3f>
PointXYZICT::getVec3Map() const noexcept {
  return Eigen::Map<const Eigen::Vector3f>(&x);
}

[[nodiscard]] Eigen::Map<Eigen::Vector4f> PointXYZICT::getVec4Map() noexcept {
  return Eigen::Map<Eigen::Vector4f>(&x);
}

[[nodiscard]] Eigen::Map<const Eigen::Vector4f>
PointXYZICT::getVec4Map() const noexcept {
  return Eigen::Map<const Eigen::Vector4f>(&x);
}

[[nodiscard]] Eigen::Vector3d PointXYZICT::getLocation() const noexcept {
  return getVec3Map().cast<double>();
}

void serialize(std::vector<uint8_t> &vec,
               const pcl_types::PointXYZICT &point) noexcept {
  serialize(vec, point.x);
  serialize(vec, point.y);
  serialize(vec, point.z);
  serialize(vec, point.intensity);
  serialize(vec, point.channel);
  serialize(vec, point.timeOffset);
}

} // namespace pcl_types
