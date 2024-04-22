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
      body_linearAcceleration{Eigen::Vector3d::Zero()}{

  // TODO: Take these parameters from OG PointLIO
  R_imu = Eigen::Matrix<double, 6, 6>::Zero();
  R_imu(0, 0) = 1e-9;
  R_imu(1, 1) = 1e-9;
  R_imu(2, 2) = 1e-9;
  R_imu(3, 3) = 1e-9;
  R_imu(4, 4) = 1e-9;
  R_imu(5, 5) = 1e-9;

  R_lidar = Eigen::Matrix<double, 3, 3>::Zero();
  R_lidar(0, 0) = 1e-9;
  R_lidar(1, 1) = 1e-9;
  R_lidar(2, 2) = 1e-9;

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

  covariance = Eigen::Matrix<double, 24, 24>::Identity();

  gen.seed(0);

}

void PointLIO::registerImu(const Imu &imu) noexcept {

  // Estimate IMU biases by assuming the robot is standing still
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

      const gtsam::Rot3 roll_R_body = compute_plane_R_vec(        // Zero yaw
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

    world_R_body_hat = world_R_body;

    // Covariance and state update
    covariance -= covariance_H_t * SLLT.solve(H) * covariance;
    
    world_R_body = world_R_body * gtsam::Rot3::Expmap(delta_state.segment<3>(0));
    world_position += delta_state.segment<3>(3);
    world_linearVelocity += delta_state.segment<3>(6);
    imuBias_gyroscope.value() += delta_state.segment<3>(9);
    imuBias_accelerometer.value() += delta_state.segment<3>(12);
    body_angularVelocity += delta_state.segment<3>(18);
    body_linearAcceleration += delta_state.segment<3>(21);

    // Estimate theta
    Eigen::Vector3d theta = gtsam::Rot3::Logmap(world_R_body*world_R_body_hat);
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

    double nlx = sampleFromGaussian(0, R_lidar(0,0));
    double nly = sampleFromGaussian(0, R_lidar(1,1));
    double nlz = sampleFromGaussian(0, R_lidar(2,2));
    
    Eigen::MatrixXd nearest_points = KDT.findNearestNeighbors(Eigen::Vector3d(point.x, point.y, point.z));
    Eigen::Vector<double, 3> plane_normal = getPlaneNormal(nearest_points);
    Eigen::Vector<double, 3> point_in_plane = nearest_points.block<1,3>(0,0); // Taking the first point from the 5 nearest points  Eigen::Vector<double, 1> residual;
    Eigen::Vector<double, 1> hl;
    hl = -plane_normal.transpose()*(world_R_body.matrix()*(Eigen::Vector3d(point.x, point.y, point.z) - Eigen::Vector3d(nlx, nly, nlz)) + world_position - point_in_plane);

    if (hl(0) <= 1e-6){
      registerPoint(point, plane_normal, point_in_plane);
    }
    else{
     KDT.build2(point);
     
    }
  }
}

void PointLIO::registerPoint(const pcl_types::PointXYZICT &point, Eigen::Vector<double, 3> plane_normal,Eigen::Vector<double, 3> point_in_plane) noexcept {
  
  propagateForwardInPlace(1); //TODO: point.stamp

  Eigen::Matrix<double, 1, 24> H;
  H.block<1, 18>(0, 6).array() = 0.0;
  H.block<1,3>(0,0) = -plane_normal.transpose()*world_R_body.matrix()*skewSymmetric(Eigen::Vector3d(point.x, point.y, point.z));
  H.block<1,3>(0,3) = plane_normal.transpose();

  Eigen::MatrixXd D(1,3);
  D = -plane_normal.transpose()*world_R_body.matrix();

  Eigen::Vector<double, 1> residual;
  residual = -plane_normal.transpose()*(world_R_body.matrix()*Eigen::Vector3d(point.x, point.y, point.z) + world_position - point_in_plane);

  const Eigen::Matrix<double, 24, 1> covariance_H_t =
      covariance * H.transpose();

  const Eigen::Matrix<double, 1, 1> S =
      H * covariance_H_t + D * R_lidar * D.transpose();

  const auto SLLT = S.llt();
  const Eigen::Vector<double, 24> delta_state =
      covariance_H_t * SLLT.solve(residual);

  world_R_body_hat = world_R_body;

  // Covariance and state update
  covariance -= covariance_H_t * SLLT.solve(H) * covariance;
  
  world_R_body = world_R_body * gtsam::Rot3::Expmap(delta_state.segment<3>(0));
  world_position += delta_state.segment<3>(3);
  world_linearVelocity += delta_state.segment<3>(6);
  imuBias_gyroscope.value() += delta_state.segment<3>(9);
  imuBias_accelerometer.value() += delta_state.segment<3>(12);
  body_angularVelocity += delta_state.segment<3>(18);
  body_linearAcceleration += delta_state.segment<3>(21);

  // Estimate theta
  Eigen::Vector3d theta = gtsam::Rot3::Logmap(world_R_body*world_R_body_hat);
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
  Fx.block<3, 3>(world_linearVelocity_index, world_R_body_index) = world_R_body.matrix()*skewSymmetric(body_linearAcceleration)*dt;
  Fx.block<3, 3>(world_linearVelocity_index, world_gravity_index) = I3dt;

  Fx.block<3, 3>(world_linearVelocity_index, body_linearAcceleration_index) =
      world_R_body.matrix() * dt;

  covariance = Fx * covariance * Fx.transpose() + Fw * Q * Fw.transpose();

  world_R_body = world_R_body * gtsam::Rot3::Expmap(body_angularVelocity * dt);
  world_position += world_linearVelocity * dt;
  world_linearVelocity +=
      (world_R_body * body_linearAcceleration + world_gravity) * dt;
}

// Eigen::Vector3d computeNormalVector(const Eigen::Vector4d& planeCoeffs) {
//     Eigen::Vector<double, 3> normalVec;
//     normalVec << planeCoeffs(0), planeCoeffs(1), planeCoeffs(2);
//     normalVec.normalize();
//     return normalVec;
// }

Eigen::Vector3d getPlaneNormal(const Eigen::MatrixXd& points) {
    // Center the points
    Eigen::Vector3d centroid = points.colwise().mean();
    Eigen::MatrixXd centered = points.rowwise() - centroid.transpose();

    // Compute SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
    Eigen::Vector3d normal = svd.matrixV().col(2); // Right singular vector corresponding to smallest singular value

    return normal;
}

  double PointLIO::sampleFromGaussian(double mean, double stddev) {
  // Function to sample a point from a Gaussian distribution

      std::normal_distribution<double> dist(mean, stddev);

      return dist(gen);
  }

} // namespace point_lio
