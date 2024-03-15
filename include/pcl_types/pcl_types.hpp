#ifndef PCL_TYPES_PCL_TYPES_HPP
#define PCL_TYPES_PCL_TYPES_HPP

#include <Eigen/Dense>

#include <cstdint>
#include <span>
#include <type_traits>
#include <vector>

namespace pcl_types {

using float32_t = std::conditional<sizeof(float) == 4, float, double>::type;
static_assert(sizeof(float32_t) == 4, "float32_t is not the right size");

using float64_t = std::conditional<sizeof(double) == 8, double, float>::type;
static_assert(sizeof(float64_t) == 8, "float64_t is not the right size");

struct PointXYZICT {
  float32_t x;
  float32_t y;
  float32_t z;
  float32_t _;
  uint16_t intensity;
  uint16_t channel;
  float32_t timeOffset;

  [[nodiscard]] Eigen::Map<Eigen::Vector<float32_t, 3>> getVec3Map() noexcept;

  [[nodiscard]] Eigen::Map<const Eigen::Vector<float32_t, 3>>
  getVec3Map() const noexcept;

  [[nodiscard]] Eigen::Map<Eigen::Vector<float32_t, 4>> getVec4Map() noexcept;

  [[nodiscard]] Eigen::Map<const Eigen::Vector<float32_t, 4>>
  getVec4Map() const noexcept;

  [[nodiscard]] Eigen::Vector3d getLocation() const noexcept;

  static constexpr size_t kX_INDEX = 0;
  static constexpr size_t kY_INDEX =
      kX_INDEX + sizeof(pcl_types::PointXYZICT::x);
  static constexpr size_t kZ_INDEX =
      kY_INDEX + sizeof(pcl_types::PointXYZICT::y);
  static constexpr size_t kINTENSITY_INDEX =
      kZ_INDEX + sizeof(pcl_types::PointXYZICT::z);
  static constexpr size_t kCHANNEL_INDEX =
      kINTENSITY_INDEX + sizeof(pcl_types::PointXYZICT::intensity);
  static constexpr size_t kTIME_OFFSET_INDEX =
      kCHANNEL_INDEX + sizeof(pcl_types::PointXYZICT::channel);
  static constexpr size_t kSIZE =
      kTIME_OFFSET_INDEX + sizeof(pcl_types::PointXYZICT::timeOffset);
};

template <typename T> struct PointCloudStamped {
  double stamp;
  std::vector<T> cloud;
};

using LidarScanStamped = PointCloudStamped<PointXYZICT>;

template <typename T>
  requires std::is_floating_point_v<T> || std::is_integral_v<T>
void serialize(std::vector<uint8_t> &vec, const T &x) noexcept {
  auto data = std::span{reinterpret_cast<const uint8_t *>(&x), sizeof(x)};
  for (size_t index = 0; index < sizeof(x); ++index) {
    vec.push_back(data[index]);
  }
}

void serialize(std::vector<uint8_t> &vec,
               const pcl_types::PointXYZICT &point) noexcept;

} // namespace pcl_types

#endif
