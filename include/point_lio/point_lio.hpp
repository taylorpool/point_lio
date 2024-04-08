#ifndef POINT_LIO_POINT_LIO_HPP
#define POINT_LIO_POINT_LIO_HPP

#include "point_lio/ekf.hpp"
#include "point_lio/incremental_kd_tree.hpp"

#include "pcl_types/pcl_types.hpp"

#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>

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

struct Imu_State {
    Eigen::Vector3d Rot; // angles (rotation)
    Eigen::Vector3d pos; // position vector
    Eigen::Vector3d V; // velocity vector
    Eigen::Vector3d bg; // gyro bias vector
    Eigen::Vector3d ba; // accelerometer bias vector
    Eigen::Vector3d g; // gravity vector
    Eigen::Vector3d angVel; // angular velocity vector
    Eigen::Vector3d linAcc; // linear acceleration vector
};

class PointLIO {
public:
  size_t m_imuInitializationQuota;
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> m_preintegrationParams;
  gtsam::NavState m_imuState;
  std::optional<gtsam::imuBias::ConstantBias> m_imuBias;
  std::deque<Imu> m_imuBuffer;
  double m_currentStamp;

  Imu_State imu_state;

  double delt;

  [[nodiscard]] PointLIO() noexcept;
  
  [[nodiscard]] gtsam::NavState registerImu(Imu_State& imu_state, const Imu &imu) noexcept;

  [[nodiscard]] std::vector<pcl_types::PointXYZICT>
  registerScan(const pcl_types::LidarScanStamped &scan) noexcept;

  // Method to convert NavState to StateInfo custom msg type
  nav_msgs::Odometry NavstateToOdometry(gtsam::NavState odometry);

  // void getImu_State(Imu_State& imu_state, const Imu &imu);

};

} // namespace point_lio

#endif
