#ifndef POINT_LIO_POINT_LIO_HPP
#define POINT_LIO_POINT_LIO_HPP

#include "point_lio/ekf.hpp"
#include "point_lio/incremental_kd_tree.hpp"

#include "pcl_types/pcl_types.hpp"

#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

#include <Eigen/Dense>

#include <chrono>
#include <optional>

namespace point_lio {

// using Stamp = std::chrono::time_point<std::chrono::system_clock,
//                                       std::chrono::nanoseconds>;
using Stamp = double;

struct Imu {
  Stamp stamp;
  Eigen::Vector3d body_measuredLinearAcceleration;
  Eigen::Vector3d body_measuredAngularVelocity;
};

class PointLIO {
public:
  size_t m_imuInitializationQuota;
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> m_preintegrationParams;
  gtsam::NavState m_imuState;
  std::optional<gtsam::imuBias::ConstantBias> m_imuBias;
  std::deque<Imu> m_imuBuffer;
  double m_currentStamp;

  [[nodiscard]] PointLIO() noexcept;
  
  [[nodiscard]] gtsam::NavState registerImu(const Imu &imu) noexcept;

  [[nodiscard]] std::vector<pcl_types::PointXYZICT>
  registerScan(const pcl_types::LidarScanStamped &scan) noexcept;
};

} // namespace point_lio

#endif
