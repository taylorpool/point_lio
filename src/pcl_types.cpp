#include "pcl_types/pcl_types.hpp"

namespace pcl_types {

void serialize(std::vector<uint8_t> &vec,
               const pcl_types::PointXYZ &point) noexcept {
  serialize(vec, point.x);
  serialize(vec, point.y);
  serialize(vec, point.z);
}

void serialize(std::vector<uint8_t> &vec,
               const pcl_types::PointXYZI &point) noexcept {
  serialize(vec, point.x);
  serialize(vec, point.y);
  serialize(vec, point.z);
  serialize(vec, point.intensity);
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

void serialize(std::vector<uint8_t> &vec, const Eigen::Vector3f data) noexcept {
  serialize(vec, data.x());
  serialize(vec, data.y());
  serialize(vec, data.z());
}

void serialize(std::vector<uint8_t> &vec, const Eigen::Vector3d data) noexcept {
  serialize(vec, data.x());
  serialize(vec, data.y());
  serialize(vec, data.z());
}

} // namespace pcl_types
