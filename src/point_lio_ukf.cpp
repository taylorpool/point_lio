#include "point_lio/point_lio_ukf.hpp"

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

// void pushBack33(std::vector<Eigen::Triplet<double>> &vec, const int row,
//                 const int col) noexcept {
//   using T = Eigen::Triplet<double>;
//   const int rowp1 = row + 1;
//   const int colp1 = col + 1;
//   const int rowp2 = row + 2;
//   const int colp2 = col + 2;

//   vec.push_back(T(row, col, 1.0));
//   vec.push_back(T(row, colp1, 1.0));
//   vec.push_back(T(row, colp2, 1.0));

//   vec.push_back(T(rowp1, col, 1.0));
//   vec.push_back(T(rowp1, colp1, 1.0));
//   vec.push_back(T(rowp1, colp2, 1.0));

//   vec.push_back(T(rowp2, col, 1.0));
//   vec.push_back(T(rowp2, colp1, 1.0));
//   vec.push_back(T(rowp2, colp2, 1.0));
// }

// void pushBackI3(std::vector<Eigen::Triplet<double>> &vec, const int row,
//                 const int col) noexcept {
//   using T = Eigen::Triplet<double>;
//   vec.push_back(T(row, col, 1.0));
//   vec.push_back(T(row + 1, col + 1, 1.0));
//   vec.push_back(T(row + 2, col + 2, 1.0));
// }

// void setFromMatrix33(Eigen::SparseMatrix<double, Eigen::RowMajor> &A,
//                      const Eigen::Matrix<double, 3, 3> &B, const int row,
//                      const int col) noexcept {
//   const int rowp1 = row + 1;
//   const int colp1 = col + 1;
//   const int rowp2 = row + 2;
//   const int colp2 = col + 2;

//   A.coeffRef(row, col) = B(0, 0);
//   A.coeffRef(row, colp1) = B(0, 1);
//   A.coeffRef(row, colp2) = B(0, 2);

//   A.coeffRef(rowp1, col) = B(1, 0);
//   A.coeffRef(rowp1, colp1) = B(1, 1);
//   A.coeffRef(rowp1, colp2) = B(1, 2);

//   A.coeffRef(rowp2, col) = B(1, 0);
//   A.coeffRef(rowp2, colp1) = B(1, 1);
//   A.coeffRef(rowp2, colp2) = B(1, 2);
// }

// void setD3FromScalar(Eigen::SparseMatrix<double, Eigen::RowMajor> &A,
//                      const double b, const int row, const int col) noexcept {
//   A.coeffRef(row, col) = b;
//   A.coeffRef(row + 1, col + 1) = b;
//   A.coeffRef(row + 2, col + 2) = b;
// }

// void initializeFx(Eigen::SparseMatrix<double, Eigen::RowMajor> &Fx) noexcept {
//   std::vector<Eigen::Triplet<double>> triplets;
//   triplets.reserve(57);

//   pushBack33(triplets, 0, 0);
//   pushBackI3(triplets, 0, 18);
//   pushBackI3(triplets, 3, 3);
//   pushBackI3(triplets, 3, 6);
//   pushBack33(triplets, 6, 0);
//   pushBackI3(triplets, 6, 6);
//   pushBackI3(triplets, 6, 15);
//   pushBack33(triplets, 6, 21);
//   pushBackI3(triplets, 9, 9);
//   pushBackI3(triplets, 12, 12);
//   pushBackI3(triplets, 15, 15);
//   pushBackI3(triplets, 18, 18);
//   pushBackI3(triplets, 21, 21);

//   Fx.setFromTriplets(triplets.cbegin(), triplets.cend());
// }

// void initializeFw(Eigen::SparseMatrix<double, Eigen::RowMajor> &Fw) noexcept {

//   std::vector<Eigen::Triplet<double>> triplets;
//   triplets.reserve(12);

//   pushBackI3(triplets, 9, 0);
//   pushBackI3(triplets, 12, 3);
//   pushBackI3(triplets, 18, 6);
//   pushBackI3(triplets, 21, 9);

//   Fw.setFromTriplets(triplets.cbegin(), triplets.cend());
// }

// void initializeH_imu(Eigen::SparseMatrix<double> &H_imu) noexcept {
//   std::vector<Eigen::Triplet<double>> triplets;
//   triplets.reserve(12);

//   pushBackI3(triplets, 0, 6);
//   pushBackI3(triplets, 0, 18);

//   H_imu.setFromTriplets(triplets.cbegin(), triplets.cend());
// }

// void initializeJ(Eigen::SparseMatrix<double, Eigen::RowMajor> &J) noexcept {
//   std::vector<Eigen::Triplet<double>> triplets;
//   triplets.reserve(9 + 21);

//   pushBack33(triplets, 0, 0);
//   pushBackI3(triplets, 3, 3);
//   pushBackI3(triplets, 6, 6);
//   pushBackI3(triplets, 9, 9);
//   pushBackI3(triplets, 12, 12);
//   pushBackI3(triplets, 15, 15);
//   pushBackI3(triplets, 18, 18);
//   pushBackI3(triplets, 21, 21);

//   J.setFromTriplets(triplets.cbegin(), triplets.cend());
// }

[[nodiscard]] PointLIO_UKF::PointLIO_UKF(const PointLIOParams &params) noexcept
    : m_params(params), stamp{0.0}, world_R_body{gtsam::Rot3::Identity()},
      world_position{Eigen::Vector3d::Zero()},
      world_linearVelocity{Eigen::Vector3d::Zero()},
      world_gravity{0.0, 0.0, -9.81},
      body_angularVelocity{Eigen::Vector3d::Zero()},
      body_linearAcceleration{Eigen::Vector3d::Zero()}, 
      m_map(params.mapParams)
      {

  // initializeFx(Fx);
  // initializeFw(Fw);
  // initializeH_imu(H_imu);
  // initializeJ(J);

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

  covariance = Eigen::Matrix<double, 22, 22>::Identity() * 1e-2;

  St = Eigen::Matrix<double, 22, 22>::Identity() * 1e-2;

  updated_covariance = Eigen::Matrix<double, 22, 22>::Identity() * 1e-2;

  sigma_mean = Eigen::Vector3d::Zero();
  meas_mean = Eigen::Vector3d::Zero(); 
  meas_pred = Eigen::Matrix<double, 22, 5>::Identity() * 1e-2;

  // Assigning updated values to state vector
  Eigen::Quaternion<double> quaternion = world_R_body.toQuaternion();
  state.segment<4>(world_R_body_index) << quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z();
  state.segment<3>(world_position_index) = world_position;
  state.segment<3>(world_linearVelocity_index) = world_linearVelocity;
  state.segment<3>(imuBias_gyroscope_index) = imuBias_gyroscope.value();
  state.segment<3>(imuBias_accelerometer_index) = imuBias_accelerometer.value();
  state.segment<3>(body_angularVelocity_index) = body_angularVelocity;
  state.segment<3>(body_linearAcceleration_index) = body_linearAcceleration;

}

void PointLIO_UKF::initializeState() noexcept {
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

  // Assigning updated values to state vector
  Eigen::Quaternion<double> quaternion = world_R_body.toQuaternion();
  state.segment<4>(world_R_body_index) << quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z();
  state.segment<3>(imuBias_gyroscope_index) = imuBias_gyroscope.value();
  state.segment<3>(imuBias_accelerometer_index) = imuBias_accelerometer.value();
  state.segment<3>(body_linearAcceleration_index) = body_linearAcceleration;

  m_imuBuffer.clear();
}

void PointLIO_UKF::statePropagateForwardInPlace(const double dt) noexcept {
  state.segment<3>(world_linearVelocity_index) += (world_R_body.rotate(state.segment<3>(body_linearAcceleration_index)) + world_gravity) * dt;
  world_R_body = world_R_body * gtsam::Rot3::Expmap(state.segment<3>(body_angularVelocity_index) * dt);
  Eigen::Quaternion<double> quat = world_R_body.toQuaternion();
  state.segment<4>(world_R_body_index) << quat.w(), quat.x(), quat.y(), quat.z();
  state.segment<3>(world_position_index) += state.segment<3>(world_linearVelocity_index) * dt;
}

void PointLIO_UKF::sigmaPropagateForwardInPlace(const double dt, Eigen::Matrix<double, 22, 5>& sigma_points) noexcept {

  int n = sigma_points.rows();        // Dimension of sigma point
  int L = sigma_points.cols();        // Number of sigma points

  for (int i = 0; i < L; ++i) {
    Eigen::Quaternion<double> q(sigma_points.block<4,1>(world_R_body_index,i));
    Eigen::Quaternion<double> rot = gtsam::Rot3::Expmap(state.segment<3>(body_angularVelocity_index) * dt).toQuaternion();
    Eigen::Quaternion<double> result = rot * q;

    sigma_points.block<4,1>(world_R_body_index,i) << result.w(), result.x(), result.y(), result.z();

    // sigma_points.block<4,1>(world_R_body_index,i) = sigma_points.block<4,1>(world_R_body_index,i) * gtsam::Rot3::Expmap(state.segment<3>(body_angularVelocity_index)* dt);
    sigma_points.block<3,1>(world_position_index,i) += state.segment<3>(world_linearVelocity_index) * dt;
    sigma_points.block<3,1>(world_linearVelocity_index,i) += (world_R_body.rotate(state.segment<3>(body_linearAcceleration_index)) + world_gravity) * dt;
  }
}

void PointLIO_UKF::covariancePropagateForwardInPlace(const double dt) noexcept {

  generateSigmaPoints(state, covariance, sigma_points, dt);
  computeMeanAndCovariance(covariance, sigma_points, sigma_mean);
}

void PointLIO_UKF::propagateForwardInPlace(const double dt) noexcept {
  covariancePropagateForwardInPlace(dt);
  statePropagateForwardInPlace(dt);
}

// void PointLIO_UKF::boxplus(const Eigen::Vector<double, 24> &deltaState) noexcept {
//   world_R_body = world_R_body *
//                  gtsam::Rot3::Expmap(deltaState.segment<3>(world_R_body_index));
//   world_position += deltaState.segment<3>(world_position_index);
//   world_linearVelocity += deltaState.segment<3>(world_linearVelocity_index);
//   imuBias_gyroscope.value() += deltaState.segment<3>(imuBias_gyroscope_index);
//   imuBias_accelerometer.value() +=
//       deltaState.segment<3>(imuBias_accelerometer_index);
//   body_angularVelocity += deltaState.segment<3>(body_angularVelocity_index);
//   body_linearAcceleration +=
//       deltaState.segment<3>(body_linearAcceleration_index);
// }

void PointLIO_UKF::registerImu(const Imu &imu) noexcept {

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

    // Predict
    double dt = imu.stamp - stamp;
    propagateForwardInPlace(dt);

    // Update
    generateSigmaPoints(sigma_mean, covariance, sigma_points, dt);

    meas_pred = sigma_points; 
    for (int i = 0; i < sigma_points.cols(); ++i) {
      meas_pred.block<3,1>(body_angularVelocity_index,i) -= sigma_points.block<3,1>(body_angularVelocity_index,i) - imuBias_gyroscope.value();
      meas_pred.block<3,1>(body_linearAcceleration_index,i) -= sigma_points.block<3,1>(body_linearAcceleration_index,i) - imuBias_accelerometer.value(); 

    computeMeanAndCovariance(St, meas_pred, meas_mean);

    computeCovariance(updated_covariance, meas_pred, sigma_points, sigma_mean, meas_mean);

    const Eigen::LDLT<Eigen::Matrix<double, 22, 22>> SLDLT = St.ldlt();

    Eigen::Matrix<double, 22, 22> K = updated_covariance * SLDLT.solve(Eigen::MatrixXd::Identity(St.rows(), St.cols()));

    Eigen::Vector<double, 22> actual_meas = meas_mean;
    actual_meas.segment<3>(body_angularVelocity_index) = imu.body_measuredAngularVelocity;
    actual_meas.segment<3>(body_linearAcceleration_index) = imu.body_measuredLinearAcceleration;

    state = sigma_mean + K*(actual_meas - meas_mean);

    covariance -= K*St*K.transpose();

    stamp = imu.stamp;
  }
  }
}

void PointLIO_UKF::registerPoint(const pcl_types::PointXYZICT &point) noexcept {
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

// Eigen::Vector3d getPlaneNormal(const Eigen::MatrixXd &points) {
//   // Center the points
//   Eigen::Vector3d centroid = points.colwise().mean();
//   Eigen::MatrixXd centered = points.rowwise() - centroid.transpose();

//   // Compute SVD
//   Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
//   Eigen::Vector3d normal = svd.matrixV().col(2); // Right singular vector corresponding to smallest singular value

//   return normal;
// }

void PointLIO_UKF::generateSigmaPoints(const Eigen::Vector<double,22>& X, const Eigen::Matrix<double, 22, 22>& covariance, Eigen::Matrix<double, 22, 5>& sigma_points, const double dt) {
    int n = X.size();  // Dimension of the state vector
    double lambda = 3 - n;  // Lambda for scaling

    // Calculate square root of (n + lambda) * P
    Eigen::MatrixXd A = (covariance * (n + lambda)).llt().matrixL();

    // Initialize sigma points matrix
    // sigma_points<n, 2*n+1>;
    sigma_points.col(0) = X;  // First sigma point is the mean

    // Generate sigma points around the mean
    for (int i = 0; i < n; ++i) {
        sigma_points.col(i + 1)     = X + A.col(i);
        sigma_points.col(n + i + 1) = X - A.col(i);
    }

    sigmaPropagateForwardInPlace(dt, sigma_points);
}

void PointLIO_UKF::computeMeanAndCovariance(Eigen::Matrix<double, 22, 22>& cov, Eigen::Matrix<double, 22, 5>& points, Eigen::Vector<double, 22>& mean) {
    int n = points.rows();        // Dimension of sigma point
    int L = points.cols();        // Number of sigma points

    const double lambda = square(alpha) * (L+kappa) - L;
    Eigen::VectorXd Wm = Eigen::VectorXd::Zero(L); 
    Eigen::VectorXd Wc = Eigen::VectorXd::Zero(L);

    // Initialize mean and covariance
    mean = Eigen::VectorXd::Zero(n);

    // Compute mean
    for (int i = 0; i < L; ++i) {
        if(i==0){
            Wm(0) = lambda/(lambda + L);
        }
        else{
            Wm(i) = 1/(2*(lambda+L));
        }

        mean += Wm(i) * points.col(i);
    }

    // Compute covariance
    for (int i = 0; i < L; ++i) {
        if(i==0){
            Wc(0) = lambda/(lambda + L) + (1-square(alpha) + beta);
        }
        else{
            Wc(i) = 1/(2*(lambda+L));
        }
        
        Eigen::VectorXd diff = points.col(i) - mean;
        cov += Wc(i) * (diff * diff.transpose());

    }
}

void PointLIO_UKF::computeCovariance(Eigen::Matrix<double, 22, 22>& updated_covariance, Eigen::Matrix<double, 22, 5>& meas_pred, Eigen::Matrix<double, 22, 5>& sigma_points, Eigen::Vector<double, 22>& sigma_mean, Eigen::Vector<double, 22>& meas_mean) {
    int n = sigma_points.rows();        // Dimension of sigma point
    int L = sigma_points.cols();        // Number of sigma points

    const double lambda = square(alpha) * (L+kappa) - L;
    Eigen::VectorXd Wm = Eigen::VectorXd::Zero(L); 
    Eigen::VectorXd Wc = Eigen::VectorXd::Zero(L);

    // Compute covariance
    for (int i = 0; i < L; ++i) {
        if(i==0){
            Wc(0) = lambda/(lambda + L) + (1-square(alpha) + beta);
        }
        else{
            Wc(i) = 1/(2*(lambda+L));
        }
        
        Eigen::VectorXd diff = sigma_points.col(i) - sigma_mean;
        Eigen::VectorXd meas_diff = meas_pred.col(i) - meas_mean;
        updated_covariance += Wc(i) * (diff * meas_diff);

    }
}
} // namespace point_lio
