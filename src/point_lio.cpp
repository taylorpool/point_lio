#include "point_lio/point_lio.hpp"

#include <algorithm>
#include <execution>

namespace point_lio {

[[nodiscard]] gtsam::Rot3
compute_plane_R_vec(const Eigen::Vector3d planeNormal,
                    const Eigen::Vector3d vec) noexcept {
  const Eigen::Vector3d y =
      (vec - vec.dot(planeNormal) * planeNormal).normalized();

  const Eigen::Vector3d x = vec.normalized();

  const double cos_theta = x.dot(y);
  const Eigen::Vector3d u_sin_theta = x.cross(y);

  return gtsam::Rot3::Quaternion(cos_theta, u_sin_theta.x(), u_sin_theta.y(),
                                 u_sin_theta.z());
}

[[nodiscard]] Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d v) {
  return (Eigen::Matrix3d() << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(),
          v.x(), 0)
      .finished();
}

[[nodiscard]] PointLIO::PointLIO() noexcept
    : m_params{.imuInitializationQuota = 10}, stamp{0.0},
      world_R_body{gtsam::Rot3::Identity()},
      world_position{Eigen::Vector3d::Zero()},
      world_linearVelocity{Eigen::Vector3d::Zero()},
      world_gravity{0.0, 0.0, -9.81},
      body_angularVelocity{Eigen::Vector3d::Zero()},
      body_linearAcceleration{Eigen::Vector3d::Zero()} {

  // TODO: Take these parameters from OG PointLIO
  R_imu = Eigen::Matrix<double, 6, 6>::Zero();
  R_imu(0, 0) = 1e-9;
  R_imu(1, 1) = 1e-9;
  R_imu(2, 2) = 1e-9;
  R_imu(3, 3) = 1e-9;
  R_imu(4, 4) = 1e-9;
  R_imu(5, 5) = 1e-9;

  Q = Eigen::Matrix<double, 12, 12>::Zero();
  Q.block<3, 3>(noise_bias_gyroscope_index, noise_bias_gyroscope_index) =
      Eigen::Matrix3d::Identity() * 0.00001;
  Q.block<3, 3>(noise_bias_accelerometer_index,
                noise_bias_accelerometer_index) =
      Eigen::Matrix3d::Identity() * 0.00001;
  Q.block<3, 3>(noise_gyroscope_index, noise_gyroscope_index) =
      Eigen::Matrix3d::Identity() * 0.00001;
  Q.block<3, 3>(noise_accelerometer_index, noise_accelerometer_index) =
      Eigen::Matrix3d::Identity() * 0.00001;

  Fw = Eigen::Matrix<double, 24, 12>::Zero();
  Fw(9, 0) = 1.0;
  Fw(10, 1) = 1.0;
  Fw(11, 2) = 1.0;

  Fw(12, 3) = 1.0;
  Fw(13, 4) = 1.0;
  Fw(14, 5) = 1.0;

  Fw(18, 6) = 1.0;
  Fw(19, 7) = 1.0;
  Fw(20, 8) = 1.0;
  Fw(21, 9) = 1.0;
  Fw(22, 10) = 1.0;
  Fw(23, 11) = 1.0;
}

void PointLIO::registerImu(const Imu &imu) noexcept {
  if (!imuBias_gyroscope || !imuBias_accelerometer) {
    if (m_imuBuffer.empty() || m_imuBuffer.back().stamp < imu.stamp) {
      m_imuBuffer.push_back(imu);
      stamp = imu.stamp;
    }
    if (m_imuBuffer.size() >= m_params.imuInitializationQuota) {
      Eigen::Vector3d body_meanMeasuredLinearAcceleration =
          Eigen::Vector3d::Zero();
      std::transform_reduce(
          std::execution::par_unseq, m_imuBuffer.cbegin(), m_imuBuffer.cend(),
          body_meanMeasuredLinearAcceleration, std::plus<Eigen::Vector3d>(),
          [this](const auto imu) {
            return imu.body_measuredLinearAcceleration / m_imuBuffer.size();
          });

      Eigen::Vector3d body_meanMeasuredAngularVelocity =
          Eigen::Vector3d::Zero();
      std::transform_reduce(
          std::execution::par_unseq, m_imuBuffer.cbegin(), m_imuBuffer.cend(),
          body_meanMeasuredAngularVelocity, std::plus<Eigen::Vector3d>(),
          [this](const auto &imu) {
            return imu.body_measuredAngularVelocity / m_imuBuffer.size();
          });

      const gtsam::Rot3 roll_R_body = compute_plane_R_vec(
          {0.0, 1.0, 0.0}, body_meanMeasuredLinearAcceleration);
      const gtsam::Rot3 world_R_roll = compute_plane_R_vec(
          {1.0, 0.0, 0.0}, roll_R_body * body_meanMeasuredLinearAcceleration);

      world_R_body = world_R_roll * roll_R_body;
      imuBias_gyroscope = body_meanMeasuredAngularVelocity;
      imuBias_accelerometer = body_meanMeasuredLinearAcceleration +
                              world_R_body.unrotate(world_gravity);
      m_imuBuffer.clear();
    }
  } else if (imu.stamp > stamp) {
    propagateForwardInPlace(imu.stamp);

    Eigen::Matrix<double, 6, 24> H;
    H.block<6, 9>(0, 0).array() = 0.0;
    H.block<6, 6>(0, 9) = Eigen::Matrix<double, 6, 6>::Identity();
    H.block<6, 3>(0, 15).array() = 0.0;
    H.block<6, 6>(0, 18) = Eigen::Matrix<double, 6, 6>::Identity();

    Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Identity();

    Eigen::Vector<double, 6> residual;
    residual.head<3>() = imu.body_measuredAngularVelocity -
                         body_angularVelocity - imuBias_gyroscope.value();
    residual.tail<3>() =
        imu.body_measuredLinearAcceleration - body_linearAcceleration -
        imuBias_accelerometer.value() + world_R_body.unrotate(world_gravity);

    const Eigen::Matrix<double, 24, 6> covariance_H_t =
        covariance * H.transpose();

    const Eigen::Matrix<double, 6, 6> S =
        H * covariance_H_t + D * R_imu * D.transpose();

    const auto SLLT = S.llt();
    const Eigen::Vector<double, 24> delta_state =
        covariance_H_t * SLLT.solve(residual);

    // TODO finish covariance and state update
    covariance -= covariance_H_t * SLLT.solve(H) * covariance;

    // TODO Estimate theta
    Eigen::Vector3d theta = gtsam::Rot3::Logmap();
    double norm_theta_div_2 = theta.norm() / 2.0;
    Eigen::Matrix<double, 24, 24> Jt;
    Jt.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity() - skewSymmetric(theta / 2.0) +
        (1.0 - norm_theta_div_2 * std::cos(norm_theta_div_2) /
                   std::sin(norm_theta_div_2)) *
            skewSymmetric(theta) / theta.norm();
    Jt.block<3, 21>(0, 3).array() = 0.0;
    Jt.block<21, 3>(3, 0).array() = 0.0;
    Jt.block<21, 21>(3, 3) = Eigen::Matrix<double, 21, 21>::Identity();
    covariance = Jt.transpose() * covariance * Jt;
  }
}

void PointLIO::registerScan(const pcl_types::LidarScanStamped &scan) noexcept {
  for (const auto &point : scan.cloud) {
    registerPoint(point);
  }
}

void PointLIO::registerPoint(const pcl_types::PointXYZICT &point) noexcept {}

void PointLIO::propagateForwardInPlace(const double _stamp) noexcept {
  const double dt = stamp - _stamp;

  const auto I3 = Eigen::Matrix3d::Identity();
  const auto I3dt = I3 * dt;

  Eigen::Matrix<double, 24, 24> Fx = Eigen::Matrix<double, 24, 24>();

  Fx.block<3, 3>(world_R_body_index, world_R_body_index) =
      gtsam::Rot3::Expmap(-body_angularVelocity * dt).matrix();
  Fx.block<3, 3>(world_R_body_index, body_angularVelocity_index) = I3dt;

  Fx.block<3, 3>(world_position_index, world_linearVelocity_index) = I3dt;

  // TODO
  Fx.block<3, 3>(world_linearVelocity_index, world_R_body_index) = I3;
  Fx.block<3, 3>(world_linearVelocity_index, world_gravity_index) = I3dt;

  Fx.block<3, 3>(world_linearVelocity_index, body_linearAcceleration_index) =
      world_R_body.matrix() * dt;

  covariance = Fx * covariance * Fx.transpose() + Fw * Q * Fw.transpose();

  world_R_body = world_R_body * gtsam::Rot3::Expmap(body_angularVelocity * dt);
  world_position += world_linearVelocity * dt;
  world_linearVelocity +=
      (world_R_body * body_linearAcceleration + world_gravity) * dt;
}

} // namespace point_lio
