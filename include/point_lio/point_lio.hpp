#ifndef POINT_LIO_POINT_LIO_HPP
#define POINT_LIO_POINT_LIO_HPP

#include "point_lio/ekf.hpp"
#include "point_lio/incremental_kd_tree.hpp"

#include "pcl_types/pcl_types.hpp"

#include "incremental_kd_tree.hpp"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

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

[[nodiscard]] Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d v);

struct PointLIOParams {
  size_t imuInitializationQuota;
};

class PointLIO {
public:
  PointLIOParams m_params;

  std::deque<Imu> m_imuBuffer;
  
  point_lio::KDTree KDT;  

  std::mt19937 gen;

  Stamp stamp;
  gtsam::Rot3 world_R_body;
  Eigen::Vector3d world_position;
  Eigen::Vector3d world_linearVelocity;
  std::optional<Eigen::Vector3d> imuBias_gyroscope;
  std::optional<Eigen::Vector3d> imuBias_accelerometer;
  Eigen::Vector3d world_gravity;
  Eigen::Vector3d body_angularVelocity;
  Eigen::Vector3d body_linearAcceleration;

  gtsam::Rot3 world_R_body_hat;

  Eigen::Matrix<double, 6, 6> R_imu;

  Eigen::Matrix<double, 3, 3> R_lidar;

  Eigen::Matrix<double, 12, 12> Q;

  Eigen::Matrix<double, 24, 12> Fw;

  Eigen::Matrix<double, 24, 24> covariance;

  // Constants
  static constexpr int world_R_body_index = 0;
  static constexpr int world_position_index = 3;
  static constexpr int world_linearVelocity_index = 6;
  static constexpr int imuBias_index = 9;
  static constexpr int world_gravity_index = 15;
  static constexpr int body_angularVelocity_index = 18;
  static constexpr int body_linearAcceleration_index = 21;

  static constexpr int noise_bias_gyroscope_index = 0;
  static constexpr int noise_bias_accelerometer_index = 3;
  static constexpr int noise_gyroscope_index = 6;
  static constexpr int noise_accelerometer_index = 9;

  // Methods
  [[nodiscard]] PointLIO() noexcept;

  void registerImu(const Imu &imu) noexcept;

  void registerScan(const pcl_types::LidarScanStamped &scan) noexcept;

  void registerPoint(const pcl_types::PointXYZICT &point, Eigen::Vector<double, 3> plane_normal,Eigen::Vector<double, 3> point_in_plane) noexcept;

  void propagateForwardInPlace(const double stamp) noexcept;

  // Eigen::Vector3d computeNormalVector(const Eigen::Vector4d& planeCoeffs);

  Eigen::Vector3d getPlaneNormal(const Eigen::MatrixXd& points);

  double sampleFromGaussian(double mean, double stddev);
};

} // namespace point_lio

#endif
