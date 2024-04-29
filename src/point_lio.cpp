#include "point_lio/point_lio.hpp"

#include <algorithm>
#include <execution>

namespace point_lio {

double square(const double x) { return x * x; }

[[nodiscard]] gtsam::Rot3
compute_plane_R_vec(const Eigen::Vector3d planeNormal,
                    const Eigen::Vector3d vec) noexcept {
  const double x_norm = vec.norm();

  const auto n_dot_vec = planeNormal.dot(vec);
  const Eigen::Vector3d y =
      (vec - n_dot_vec * planeNormal).normalized() * x_norm;

  const double cos_theta = vec.dot(y) / square(x_norm);

  const double sin_theta = std::sqrt(1.0d - square(cos_theta)) *
                           (2 * static_cast<double>(n_dot_vec > 0) - 1.0d);

  const Eigen::Vector3d axis = planeNormal.cross(vec).normalized();

  return gtsam::Rot3::Quaternion(cos_theta, sin_theta * axis.x(),
                                 sin_theta * axis.y(), sin_theta * axis.z());
}

[[nodiscard]] Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d v) noexcept {
  return (Eigen::Matrix3d() << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(),
          v.x(), 0)
      .finished();
}

void pushBack33(std::vector<Eigen::Triplet<double>> &vec, const int row,
                const int col) noexcept {
  using T = Eigen::Triplet<double>;
  const int rowp1 = row + 1;
  const int colp1 = col + 1;
  const int rowp2 = row + 2;
  const int colp2 = col + 2;

  vec.push_back(T(row, col, 1.0));
  vec.push_back(T(row, colp1, 1.0));
  vec.push_back(T(row, colp2, 1.0));

  vec.push_back(T(rowp1, col, 1.0));
  vec.push_back(T(rowp1, colp1, 1.0));
  vec.push_back(T(rowp1, colp2, 1.0));

  vec.push_back(T(rowp2, col, 1.0));
  vec.push_back(T(rowp2, colp1, 1.0));
  vec.push_back(T(rowp2, colp2, 1.0));
}

void pushBackI3(std::vector<Eigen::Triplet<double>> &vec, const int row,
                const int col) noexcept {
  using T = Eigen::Triplet<double>;
  vec.push_back(T(row, col, 1.0));
  vec.push_back(T(row + 1, col + 1, 1.0));
  vec.push_back(T(row + 2, col + 2, 1.0));
}

void setFromMatrix33(Eigen::SparseMatrix<double, Eigen::RowMajor> &A,
                     const Eigen::Matrix<double, 3, 3> &B, const int row,
                     const int col) noexcept {
  const int rowp1 = row + 1;
  const int colp1 = col + 1;
  const int rowp2 = row + 2;
  const int colp2 = col + 2;

  A.coeffRef(row, col) = B(0, 0);
  A.coeffRef(row, colp1) = B(0, 1);
  A.coeffRef(row, colp2) = B(0, 2);

  A.coeffRef(rowp1, col) = B(1, 0);
  A.coeffRef(rowp1, colp1) = B(1, 1);
  A.coeffRef(rowp1, colp2) = B(1, 2);

  A.coeffRef(rowp2, col) = B(1, 0);
  A.coeffRef(rowp2, colp1) = B(1, 1);
  A.coeffRef(rowp2, colp2) = B(1, 2);
}

void setD3FromScalar(Eigen::SparseMatrix<double, Eigen::RowMajor> &A,
                     const double b, const int row, const int col) noexcept {
  A.coeffRef(row, col) = b;
  A.coeffRef(row + 1, col + 1) = b;
  A.coeffRef(row + 2, col + 2) = b;
}

void initializeFx(Eigen::SparseMatrix<double, Eigen::RowMajor> &Fx) noexcept {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(57);

  pushBack33(triplets, 0, 0);
  pushBackI3(triplets, 0, 18);
  pushBackI3(triplets, 3, 3);
  pushBackI3(triplets, 3, 6);
  pushBack33(triplets, 6, 0);
  pushBackI3(triplets, 6, 6);
  pushBackI3(triplets, 6, 15);
  pushBack33(triplets, 6, 21);
  pushBackI3(triplets, 9, 9);
  pushBackI3(triplets, 12, 12);
  pushBackI3(triplets, 15, 15);
  pushBackI3(triplets, 18, 18);
  pushBackI3(triplets, 21, 21);

  Fx.setFromTriplets(triplets.cbegin(), triplets.cend());
}

void initializeFw(Eigen::SparseMatrix<double, Eigen::RowMajor> &Fw) noexcept {

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(12);

  pushBackI3(triplets, 9, 0);
  pushBackI3(triplets, 12, 3);
  pushBackI3(triplets, 18, 6);
  pushBackI3(triplets, 21, 9);

  Fw.setFromTriplets(triplets.cbegin(), triplets.cend());
}

void initializeH_imu(Eigen::SparseMatrix<double> &H_imu) noexcept {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(12);

  pushBackI3(triplets, 0, 6);
  pushBackI3(triplets, 0, 18);

  H_imu.setFromTriplets(triplets.cbegin(), triplets.cend());
}

void initializeJ(Eigen::SparseMatrix<double, Eigen::RowMajor> &J) noexcept {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(9 + 21);

  pushBack33(triplets, 0, 0);
  pushBackI3(triplets, 3, 3);
  pushBackI3(triplets, 6, 6);
  pushBackI3(triplets, 9, 9);
  pushBackI3(triplets, 12, 12);
  pushBackI3(triplets, 15, 15);
  pushBackI3(triplets, 18, 18);
  pushBackI3(triplets, 21, 21);

  J.setFromTriplets(triplets.cbegin(), triplets.cend());
}

[[nodiscard]] PointLIO::PointLIO(const PointLIOParams &params) noexcept
    : m_params(params), stamp{0.0}, world_R_body{gtsam::Rot3::Identity()},
      world_position{Eigen::Vector3d::Zero()},
      world_linearVelocity{Eigen::Vector3d::Zero()},
      world_gravity{0.0, 0.0, -9.81},
      body_angularVelocity{Eigen::Vector3d::Zero()},
      body_linearAcceleration{Eigen::Vector3d::Zero()}, Fx(24, 24), Fw(24, 12),
      H_imu(6, 24), J(24, 24), m_map(params.mapParams) {

  initializeFx(Fx);
  initializeFw(Fw);
  initializeH_imu(H_imu);
  initializeJ(J);

  // TODO: Take these parameters from OG PointLIO
  R_imu = Eigen::Matrix<double, 6, 6>::Identity() * 1e-2;

  R_lidar = Eigen::Matrix<double, 3, 3>::Identity() * 1e-2;

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

  covariance = Eigen::Matrix<double, 24, 24>::Identity() * 1e-2;
}

void PointLIO::initializeState() noexcept {
  Eigen::Vector3d body_meanMeasuredLinearAcceleration = Eigen::Vector3d::Zero();

  body_meanMeasuredLinearAcceleration = std::transform_reduce(
      std::execution::par_unseq, m_imuBuffer.cbegin(), m_imuBuffer.cend(),
      body_meanMeasuredLinearAcceleration, std::plus<Eigen::Vector3d>(),
      [this](const auto &imu) {
        return imu.body_measuredLinearAcceleration / m_imuBuffer.size();
      });

  Eigen::Vector3d body_meanMeasuredAngularVelocity = Eigen::Vector3d::Zero();

  body_meanMeasuredAngularVelocity = std::transform_reduce(
      std::execution::par_unseq, m_imuBuffer.cbegin(), m_imuBuffer.cend(),
      body_meanMeasuredAngularVelocity, std::plus<Eigen::Vector3d>(),
      [this](const auto &imu) {
        return imu.body_measuredAngularVelocity / m_imuBuffer.size();
      });

  const gtsam::Rot3 roll_R_body = compute_plane_R_vec( // Zero yaw
      {0.0, 1.0, 0.0}, body_meanMeasuredLinearAcceleration);
  const gtsam::Rot3 world_R_roll = compute_plane_R_vec(
      {1.0, 0.0, 0.0}, roll_R_body * body_meanMeasuredLinearAcceleration);

  world_R_body = world_R_roll * roll_R_body;
  imuBias_gyroscope = body_meanMeasuredAngularVelocity;
  body_linearAcceleration = -world_R_body.unrotate(world_gravity);
  imuBias_accelerometer =
      body_meanMeasuredLinearAcceleration - body_linearAcceleration;

  m_imuBuffer.clear();
}

void PointLIO::statePropagateForwardInPlace(const double dt) noexcept {
  world_linearVelocity +=
      (world_R_body.rotate(body_linearAcceleration) + world_gravity) * dt;
  world_R_body = world_R_body * gtsam::Rot3::Expmap(body_angularVelocity * dt);
  world_position += world_linearVelocity * dt;
}

void PointLIO::covariancePropagateForwardInPlace(const double dt) noexcept {
  {
    const Eigen::Matrix<double, 3, 3> F11 =
        gtsam::Rot3::Expmap(-body_angularVelocity * dt).matrix();
    setFromMatrix33(Fx, F11, 0, 0);
  }
  setD3FromScalar(Fx, dt, 18, 0);
  setD3FromScalar(Fx, dt, 3, 6);
  setD3FromScalar(Fx, dt, 6, 15);
  {
    const Eigen::Matrix<double, 3, 3> F38 = world_R_body.matrix() * dt;
    const Eigen::Matrix<double, 3, 3> F31 =
        -F38 * skewSymmetric(body_linearAcceleration);
    setFromMatrix33(Fx, F31, 6, 0);
    setFromMatrix33(Fx, F38, 6, 21);
  }
  covariance = Fx * covariance * Fx.transpose() + Fw * Q * Fw.transpose();
}

void PointLIO::propagateForwardInPlace(const double dt) noexcept {
  covariancePropagateForwardInPlace(dt);
  statePropagateForwardInPlace(dt);
}

void PointLIO::boxplus(const Eigen::Vector<double, 24> &deltaState) noexcept {
  world_R_body = world_R_body *
                 gtsam::Rot3::Expmap(deltaState.segment<3>(world_R_body_index));
  world_position += deltaState.segment<3>(world_position_index);
  world_linearVelocity += deltaState.segment<3>(world_linearVelocity_index);
  imuBias_gyroscope.value() += deltaState.segment<3>(imuBias_gyroscope_index);
  imuBias_accelerometer.value() +=
      deltaState.segment<3>(imuBias_accelerometer_index);
  body_angularVelocity += deltaState.segment<3>(body_angularVelocity_index);
  body_linearAcceleration +=
      deltaState.segment<3>(body_linearAcceleration_index);
}

void PointLIO::registerImu(const Imu &imu) noexcept {

  // Estimate IMU biases by assuming the robot is standing still
  if (!imuBias_gyroscope || !imuBias_accelerometer) {
    if (m_imuBuffer.empty() || m_imuBuffer.back().stamp < imu.stamp) {
      m_imuBuffer.push_back(imu);
      stamp = imu.stamp;
    }
    if (m_imuBuffer.size() >= m_params.imuInitializationQuota) {
      initializeState();
    }
  } else if (imu.stamp > stamp) {
    propagateForwardInPlace(imu.stamp - stamp);

    Eigen::Vector<double, 6> residual;
    residual.head<3>() = imu.body_measuredAngularVelocity -
                         body_angularVelocity - imuBias_gyroscope.value();
    residual.tail<3>() = imu.body_measuredLinearAcceleration -
                         body_linearAcceleration -
                         imuBias_accelerometer.value();

    const Eigen::Matrix<double, 24, 6> covariance_H_t =
        covariance * H_imu.transpose();

    const Eigen::Matrix<double, 6, 6> S = H_imu * covariance_H_t + R_imu;

    const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> SLDLT = S.ldlt();

    const Eigen::Vector<double, 24> deltaState =
        covariance_H_t * SLDLT.solve(residual);

    boxplus(deltaState);

    // Estimate theta
    const Eigen::Vector3d theta = deltaState.segment<3>(0);

    double norm_theta_div_2 = theta.norm() / 2.0;

    const Eigen::Matrix<double, 3, 3> A =
        Eigen::Matrix3d::Identity() - skewSymmetric(theta / 2.0) +
        (1.0 - norm_theta_div_2 * std::cos(norm_theta_div_2) /
                   std::sin(norm_theta_div_2)) *
            skewSymmetric(theta) / theta.norm();
    setFromMatrix33(J, A.inverse().transpose(), 0, 0);

    covariance =
        J * (covariance - covariance_H_t * SLDLT.solve(H_imu * covariance)) *
        J.transpose();

    stamp = imu.stamp;
  }
}

void PointLIO::registerPoint(const pcl_types::PointXYZICT &point) noexcept {
  if (!imuBias_accelerometer || !imuBias_gyroscope) {
    return;
  }

  propagateForwardInPlace(point.timeOffset - stamp);

  // const Eigen::Vector3d world_point =
  //     world_R_body * point.getVec3Map().cast<double>() + world_position;

  // const Eigen::Matrix<double, 5, 3> nearest_points =
  //     KDT.findNearestNeighbors(world_point);

  // const Eigen::Vector<double, 3> plane_normal =
  // getPlaneNormal(nearest_points); const Eigen::Vector<double, 3>
  // point_in_plane =
  //     nearest_points.row(0).transpose();

  // Eigen::Vector<double, 1> hl;
  // hl = -plane_normal.transpose() *
  //      (world_R_body * world_point + world_position - point_in_plane);
  // if (abs(hl(0)) > 0.1) {
  //   KDT.build2(point.getVec3Map().cast<double>());
  // }

  // const Eigen::Matrix<double, 1, 3> D_lidar =
  //     -plane_normal.transpose() * world_R_body.matrix();

  // Eigen::Matrix<double, 1, 24> H_lidar;
  // H_lidar.block<1, 3>(0, 0) =
  //     D_lidar * skewSymmetric(Eigen::Vector3d(point.x, point.y, point.z));
  // H_lidar.block<1, 3>(0, 3) = plane_normal.transpose();
  // H_lidar.block<1, 18>(0, 6).array() = 0.0;

  // const Eigen::Vector<double, 1> residual = plane_normal.dot(
  //     world_R_body * point.getVec3Map() + world_position - plane_centroid);

  // const Eigen::Matrix<double, 24, 1> covariance_H_t =
  //     covariance * H_lidar.transpose();

  // const Eigen::Matrix<double, 6, 6> S =
  //     H_lidar * covariance_H_t + D_lidar * R_lidar * D_lidar.transpose();

  // const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> SLDLT = S.ldlt();

  // const Eigen::Vector<double, 24> deltaState =
  //     covariance_H_t * SLDLT.solve(residual);

  // boxplus(deltaState);

  // // Estimate theta
  // const Eigen::Vector3d theta = deltaState.segment<3>(0);

  // double norm_theta_div_2 = theta.norm() / 2.0;

  // const Eigen::Matrix<double, 3, 3> A =
  //     Eigen::Matrix3d::Identity() - skewSymmetric(theta / 2.0) +
  //     (1.0 - norm_theta_div_2 * std::cos(norm_theta_div_2) /
  //                std::sin(norm_theta_div_2)) *
  //         skewSymmetric(theta) / theta.norm();
  // setFromMatrix33(J, A.inverse().transpose(), 0, 0);

  // covariance =
  //     J * (covariance - covariance_H_t * SLDLT.solve(H_lidar * covariance)) *
  //     J.transpose();

  stamp = point.timeOffset;
}

// Eigen::Vector3d computeNormalVector(const Eigen::Vector4d& planeCoeffs) {
//     Eigen::Vector<double, 3> normalVec;
//     normalVec << planeCoeffs(0), planeCoeffs(1), planeCoeffs(2);
//     normalVec.normalize();
//     return normalVec;
// }

Eigen::Vector3d getPlaneNormal(const Eigen::MatrixXd &points) {
  // Center the points
  Eigen::Vector3d centroid = points.colwise().mean();
  Eigen::MatrixXd centered = points.rowwise() - centroid.transpose();

  // Compute SVD
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
  Eigen::Vector3d normal = svd.matrixV().col(
      2); // Right singular vector corresponding to smallest singular value

  return normal;
}

} // namespace point_lio
