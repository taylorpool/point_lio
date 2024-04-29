#ifndef ULTRA_ODOMETRY_VOXEL_GRID_HPP
#define ULTRA_ODOMETRY_VOXEL_GRID_HPP

#include <pcl_types/pcl_types.hpp>

#include <Eigen/Dense>

#include <deque>
#include <iostream>
#include <optional>
#include <ranges>
#include <unordered_map>

namespace ultra_odometry {

struct VoxelGridParams {
  double voxelSize;
  size_t maxPointsPerVoxel;
  int64_t voxelSearchRadius;
};

[[nodiscard]] constexpr size_t cantor(size_t x, size_t y) noexcept {
  return ((x + y) * (x + y + 1)) / static_cast<size_t>(2) + y;
}

[[nodiscard]] constexpr size_t doubleCantor(size_t x, size_t y,
                                            size_t z) noexcept {
  return cantor(x, cantor(y, z));
}

template <pcl_types::PointT Point> class VoxelGrid {
private:
  VoxelGridParams m_params;
  size_t m_numPoints;
  std::unordered_map<size_t, std::deque<Point>> m_map;

  [[nodiscard]] inline Eigen::Array<int64_t, 3, 1>
  computeVoxelCoordinates(const Point &point) const noexcept {
    return (point.getArray3fMap().template cast<double>() / m_params.voxelSize)
        .round()
        .template cast<int64_t>();
  }

  [[nodiscard]] constexpr size_t
  computeVoxelHash(const int64_t x, const int64_t y,
                   const int64_t z) const noexcept {
    const size_t offset = 1000000;
    return doubleCantor(static_cast<size_t>(x + offset),
                        static_cast<size_t>(y + offset),
                        static_cast<size_t>(z + offset));
  }

  [[nodiscard]] inline size_t computeVoxelHash(
      const Eigen::Array<int64_t, 3, 1> &voxelCoordinates) const noexcept {
    return computeVoxelHash(voxelCoordinates.x(), voxelCoordinates.y(),
                            voxelCoordinates.z());
  }

  [[nodiscard]] inline size_t
  computeVoxelHash(const Point &point) const noexcept {
    return computeVoxelHash(computeVoxelCoordinates(point));
  }

public:
  [[nodiscard]] VoxelGrid(const VoxelGridParams &params) noexcept
      : m_params(params), m_numPoints(0) {}

  [[nodiscard]] VoxelGrid(const VoxelGridParams &params,
                          const std::vector<Point> &points) noexcept
      : m_params(params), m_numPoints(0) {
    insert(points);
  }

  void insert(const Point &point) noexcept {
    if (!point.getVector3fMap().allFinite()) {
      return;
    }
    const auto voxelHash = computeVoxelHash(point);

    if (m_map.count(voxelHash) == 1) {
      auto &voxel = m_map[voxelHash];
      if (voxel.size() < m_params.maxPointsPerVoxel) {
        m_map[voxelHash].push_back(point);
        ++m_numPoints;
      }
    } else {
      m_map[voxelHash] = std::deque<Point>{point};
      ++m_numPoints;
    }
  }

  template <typename R>
    requires(std::ranges::input_range<R> &&
             std::is_same_v<Point, std::ranges::range_value_t<R>>)
  void insert(const R &points) noexcept {
    for (const auto &point : points) {
      insert(point);
    }
  }

  struct SearchResult {
    Point point;
    pcl_types::element_value_t<Point> squaredDistance;
  };

  [[nodiscard]] std::optional<SearchResult>
  findClosestTo(const Point &point) const noexcept {
    const auto voxelCoordinates = computeVoxelCoordinates(point);
    std::optional<SearchResult> result;

    for (auto x = voxelCoordinates.x() - m_params.voxelSearchRadius;
         x < voxelCoordinates.x() + m_params.voxelSearchRadius; ++x) {
      for (auto y = voxelCoordinates.y() - m_params.voxelSearchRadius;
           y < voxelCoordinates.y() + m_params.voxelSearchRadius; ++y) {
        for (auto z = voxelCoordinates.z() - m_params.voxelSearchRadius;
             z < voxelCoordinates.z() + m_params.voxelSearchRadius; ++z) {
          const auto voxelHash = computeVoxelHash(x, y, z);
          if (m_map.count(voxelHash) == 1) {
            for (const auto &tmpPoint : m_map.at(voxelHash)) {
              const auto tmpSquaredDistance =
                  (tmpPoint.getVector3fMap() - point.getVector3fMap())
                      .squaredNorm();
              if (!result || tmpSquaredDistance < result->squaredDistance) {
                result = SearchResult{.point = tmpPoint,
                                      .squaredDistance = tmpSquaredDistance};
              }
            }
          }
        }
      }
    }

    return result;
  }

  [[nodiscard]] std::vector<Point>
  findClosestTo(const Point &point, const size_t numPoints) const noexcept {
    const auto voxelCoordinates = computeVoxelCoordinates(point);
    std::vector<SearchResult> result;
    result.reserve(numPoints);

    for (auto x = voxelCoordinates.x() - m_params.voxelSearchRadius;
         x < voxelCoordinates.x() + m_params.voxelSearchRadius; ++x) {
      for (auto y = voxelCoordinates.y() - m_params.voxelSearchRadius;
           y < voxelCoordinates.y() + m_params.voxelSearchRadius; ++y) {
        for (auto z = voxelCoordinates.z() - m_params.voxelSearchRadius;
             z < voxelCoordinates.z() + m_params.voxelSearchRadius; ++z) {
          const auto voxelHash = computeVoxelHash(x, y, z);
          if (m_map.count(voxelHash) == 1) {
            for (const auto &tmpPoint : m_map.at(voxelHash)) {
              const auto tmpSquaredDistance =
                  (tmpPoint.getVector3fMap() - point.getVector3fMap())
                      .squaredNorm();
              if (result.size() < numPoints ||
                  tmpSquaredDistance < result.back().squaredDistance) {
                result.push_back(
                    {.point = tmpPoint, .squaredDistance = tmpSquaredDistance});
                std::sort(
                    result.begin(), result.end(),
                    [](const SearchResult &a, const SearchResult &b) -> bool {
                      return a.squaredDistance < b.squaredDistance;
                    });
                if (result.size() > numPoints) {
                  result.pop_back();
                }
              }
            }
          }
        }
      }
    }
    return result;
  }

  [[nodiscard]] constexpr size_t size() const noexcept { return m_numPoints; }

  [[nodiscard]] constexpr size_t numPoints() const noexcept {
    return m_numPoints;
  }

  [[nodiscard]] constexpr bool empty() const noexcept {
    return m_numPoints == 0;
  }

  [[nodiscard]] constexpr auto elements() const noexcept {
    return m_map | std::ranges::views::values | std::ranges::views::join;
  }

  // TODO make this for arbitrary range
  [[nodiscard]] std::vector<Point> toVector() const noexcept {
    std::vector<Point> points;
    points.reserve(m_numPoints);
    for (const auto &[hash, voxel] : m_map) {
      for (const auto &point : voxel) {
        points.push_back(point);
      }
    }
    return points;
  }
};

template <pcl_types::PointT Point>
[[nodiscard]] std::vector<Point> downsample(const std::vector<Point> &points,
                                            const double voxelSize,
                                            const size_t maxPointsPerVoxel) {
  return VoxelGrid(VoxelGridParams{.voxelSize = voxelSize,
                                   .maxPointsPerVoxel = maxPointsPerVoxel,
                                   .voxelSearchRadius = 0},
                   points)
      .toVector();
}

} // namespace ultra_odometry

#endif
