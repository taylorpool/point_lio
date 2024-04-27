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

#define __PCL_TYPES_XYZ__                                                      \
  float32_t x;                                                                 \
  float32_t y;                                                                 \
  float32_t z;                                                                 \
  float32_t _ = 0;                                                             \
  [[nodiscard]] inline Eigen::Map<Eigen::Vector<float32_t, 3>>                 \
  getVector3fMap() noexcept {                                                  \
    return Eigen::Map<Eigen::Vector<float32_t, 3>>(&x);                        \
  }                                                                            \
  [[nodiscard]] inline Eigen::Map<const Eigen::Vector<float32_t, 3>>           \
  getVector3fMap() const noexcept {                                            \
    return Eigen::Map<const Eigen::Vector<float32_t, 3>>(&x);                  \
  }                                                                            \
  [[nodiscard]] inline Eigen::Map<Eigen::Array<float32_t, 3, 1>>               \
  getArray3fMap() noexcept {                                                   \
    return Eigen::Map<Eigen::Array<float32_t, 3, 1>>(&x);                      \
  }                                                                            \
  [[nodiscard]] inline Eigen::Map<const Eigen::Array<float32_t, 3, 1>>         \
  getArray3fMap() const noexcept {                                             \
    return Eigen::Map<const Eigen::Array<float32_t, 3, 1>>(&x);                \
  }                                                                            \
  [[nodiscard]] inline Eigen::Map<const Eigen::Vector<float32_t, 4>>           \
  getVector4fMap() const noexcept {                                            \
    return Eigen::Map<const Eigen::Vector<float32_t, 4>>(&x);                  \
  }                                                                            \
  [[nodiscard]] inline Eigen::Vector3d getLocation() const noexcept {          \
    return getVector3fMap().cast<double>();                                    \
  }                                                                            \
  static constexpr size_t kX_INDEX = 0;                                        \
  static constexpr size_t kY_INDEX = 4;                                        \
  static constexpr size_t kZ_INDEX = 8;

#define __PCL_TYPES_XYZI__                                                     \
  __PCL_TYPES_XYZ__                                                            \
  uint16_t intensity;                                                          \
  static constexpr size_t kINTENSITY_INDEX = 12;

#define __PCL_TYPES_XYZIC__                                                    \
  __PCL_TYPES_XYZI__                                                           \
  uint16_t channel;                                                            \
  static constexpr size_t kCHANNEL_INDEX = 14;

template <typename T>
concept PointT = requires(T x) {
  {
    x.getVector3fMap()
  } -> std::same_as<Eigen::Map<Eigen::Vector<float32_t, 3>>>;
  {
    x.getVector4fMap()
  } -> std::same_as<Eigen::Map<const Eigen::Vector<float32_t, 4>>>;
  {
    x.getArray3fMap()
  } -> std::same_as<Eigen::Map<Eigen::Array<float32_t, 3, 1>>>;
};

template <PointT Point> struct element_value {
  using type = float32_t;
};

template <PointT Point> using element_value_t = element_value<Point>::type;

struct PointXYZ {
  __PCL_TYPES_XYZ__
};

static_assert(PointT<PointXYZ>);

struct PointXYZI {
  __PCL_TYPES_XYZI__
};

static_assert(PointT<PointXYZI>);

struct PointXYZIC {
  __PCL_TYPES_XYZIC__
};

static_assert(PointT<PointXYZIC>);

struct PointXYZICT {
  __PCL_TYPES_XYZIC__

  float32_t timeOffset;

  static constexpr size_t kTIME_OFFSET_INDEX =
      kCHANNEL_INDEX + sizeof(pcl_types::PointXYZICT::channel);
  static constexpr size_t kSIZE =
      kTIME_OFFSET_INDEX + sizeof(pcl_types::PointXYZICT::timeOffset);
};

static_assert(PointT<PointXYZICT>);

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
               const pcl_types::PointXYZ &point) noexcept;

void serialize(std::vector<uint8_t> &vec,
               const pcl_types::PointXYZI &point) noexcept;

void serialize(std::vector<uint8_t> &vec,
               const pcl_types::PointXYZICT &point) noexcept;

void serialize(std::vector<uint8_t> &vec, const Eigen::Vector3f data) noexcept;

void serialize(std::vector<uint8_t> &vec, const Eigen::Vector3d data) noexcept;

} // namespace pcl_types

#endif
