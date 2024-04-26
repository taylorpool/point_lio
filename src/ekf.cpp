// #include "point_lio/ekf.hpp"

// namespace point_lio {

//     // TODO: initialization of P, X, Q, Ra, Rg, Rl
//     EKF::EKF() {
//         P.setZero();
//         state.Rot.setZero();
//         state.pos.setZero();
//         state.V.setZero();
//         state.bg.setZero();
//         state.ba.setZero();
//         state.g << 0.0, 0.0, -9.81;
//         state.angVel.setZero();
//         state.linAcc.setZero();

//         gen.seed(0);
//     };

//     double EKF::sampleFromGaussian(double mean, double stddev) { // to define
//     this only once
//     // Function to sample a point from a Gaussian distribution

//         std::normal_distribution<double> dist(mean, stddev);

//         return dist(gen);
//     }

//     Eigen::Matrix3d EKF::skewSymmetric(const Eigen::Vector3d& v) {
//         Eigen::Matrix3d skew;
//         skew <<  0, -v[2],  v[1],
//                 v[2],  0,  -v[0],
//                 -v[1],  v[0], 0;
//         return skew;
//     }

//     // Eigen::Matrix3d EKF::eulerToRotationMatrix(Eigen::Vector3d& euler) {
//     //     double roll = euler(0);
//     //     double pitch = euler(1);
//     //     double yaw = euler(2);

//     //     Eigen::Matrix3d rotationMatrix;
//     //     rotationMatrix << cos(yaw)*cos(pitch),
//     cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll),
//     cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll),
//     //                     sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)
//     + cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll),
//     //                     -sin(pitch),          cos(pitch)*sin(roll),
//     cos(pitch)*cos(roll);
//     //     return rotationMatrix;
//     // }

//     Eigen::VectorXd EKF::stateToVector(EKF::State& state) {
//         Eigen::VectorXd X(24);
//         X << state.Rot, state.pos, state.V, state.bg, state.ba, state.g,
//         state.angVel, state.linAcc; return X;
//     }

//     EKF::State EKF::vectorToState(Eigen::VectorXd& X) {

//         State state;
//         state.Rot = X.block<3, 1>(0, 0);
//         state.pos = X.block<3, 1>(3, 0);
//         state.V = X.block<3, 1>(6, 0);
//         state.bg = X.block<3, 1>(9, 0);
//         state.ba = X.block<3, 1>(12, 0);
//         state.g = X.block<3, 1>(15, 0);
//         state.angVel = X.block<3, 1>(18, 0);
//         state.linAcc = X.block<3, 1>(21, 0);
//         return state;
//     }

//     void EKF::predict(EKF::State& state, double dt, Eigen::MatrixXd& P)  {
//     // Predicts the state vector X and covariance P (w/o measurement)

//         // Initializing dynamic function f as X_dot
//         Eigen::VectorXd X_dot(24);

//         // defines [angVel]
//         Eigen::Matrix3d angVel_skew = skewSymmetric(state.angVel);

//         // defines noises and biases
//         Eigen::Vector3d nbg;
//         double nbgx;
//         double nbgy;
//         double nbgz;
//         Eigen::Vector3d nba;
//         double nbax;
//         double nbay;
//         double nbaz;
//         Eigen::Vector3d wg;
//         double wgx;
//         double wgy;
//         double wgz;
//         Eigen::Vector3d wa;
//         double wax;
//         double way;
//         double waz;

//         Eigen::Matrix3d Qbg = Q.block<3,3>(0,0);
//         Eigen::Matrix3d Qba = Q.block<3,3>(3,3);
//         Eigen::Matrix3d Qg = Q.block<3,3>(6,6);
//         Eigen::Matrix3d Qa = Q.block<3,3>(9,9);

//         // Assuming uncorrelated?
//         nbgx = sampleFromGaussian(0, Qbg(0,0));
//         nbgy = sampleFromGaussian(0, Qbg(1,1));
//         nbgz = sampleFromGaussian(0, Qbg(2,2));
//         nbg << nbgx, nbgy, nbgz;

//         nbax = sampleFromGaussian(0, Qba(0,0));
//         nbay = sampleFromGaussian(0, Qba(1,1));
//         nbaz = sampleFromGaussian(0, Qba(2,2));
//         nba << nbax, nbay, nbaz;

//         wgx = sampleFromGaussian(0, Qg(0,0));
//         wgy = sampleFromGaussian(0, Qg(1,1));
//         wgz = sampleFromGaussian(0, Qg(2,2));
//         wg << wgx, wgy, wgz;

//         wax = sampleFromGaussian(0, Qa(0,0));
//         way = sampleFromGaussian(0, Qa(1,1));
//         waz = sampleFromGaussian(0, Qa(2,2));
//         wa << wax, way, waz;

//         // Rotation(k+1) converted from (3,1) to (3,1) back to (3,1)
//         // to be directly used X(k+1)
//         // Eigen::Matrix3d Rotation;
//         // Rotation = eulerToRotationMatrix(state.Rot);
//         // Rotation += (Rotation*angVel_skew)*dt;
//         // Eigen::Vector3d euler_angles = Rotation.eulerAngles(0, 1, 2);  //
//         remove euler angles double yaw = state.Rot(0); double pitch =
//         state.Rot(1); double roll = state.Rot(2); gtsam::Rot3 Rot_object =
//         gtsam::Rot3::RzRyRx(yaw, pitch, roll);
//         // Eigen::Matrix3d Rotation = Rot_object.matrix();

//         // Setting X_dot
//         X_dot.segment<3>(3) = state.V;
//         X_dot.segment<3>(6) = Rot_object.matrix()*state.linAcc + state.g;
//         X_dot.segment<3>(9) = nbg;
//         X_dot.segment<3>(12) = nba;
//         X_dot.segment<3>(15) = Eigen::Vector3d::Zero();
//         X_dot.segment<3>(18) = wg;
//         X_dot.segment<3>(21) = wa;

//         // Rotation at next step
//         Rot_object.matrix() += (Rot_object.matrix()*angVel_skew)*dt;
//         gtsam::Vector3 euler_angles = Rot_object.rpy();

//         // Predict the state X(k+1) - discretized model
//         Eigen::VectorXd X(24);
//         X = stateToVector(state);
//         X.segment<21>(3) += dt*X_dot.segment<21>(3);
//         X.segment<3>(0) = euler_angles;

//         // Prediction matrix Fx and process noise matrix Fw
//         Eigen::MatrixXd Fx = Eigen::MatrixXd::Identity(24, 24);
//         Eigen::MatrixXd Fw = Eigen::MatrixXd::Zero(24, 12);

//         // Setting Fx
//         // TODO: define F31
//         const gtsam::Rot3 exp = gtsam::Rot3::Expmap(state.angVel*dt);

//         Fx.block<3,3>(0,0) = exp.matrix();
//         Fx.block<3,3>(0,18) = dt*Eigen::MatrixXd::Identity(3,3);
//         Fx.block<3,3>(3,6) = dt*Eigen::MatrixXd::Identity(3,3);
//         //Fx.block<3,3>(6,0) = F31;
//         Fx.block<3,3>(6,15) = dt*Eigen::MatrixXd::Identity(3,3);
//         Fx.block<3,3>(6,21) = Rot_object.matrix()*dt;

//         // Setting Fw
//         Fw.block<6, 6>(9, 0) = Eigen::MatrixXd::Identity(6, 6);
//         Fw.block<6, 6>(18, 6) = Eigen::MatrixXd::Identity(6, 6);

//         // Update covariance matrix P -
//         P = Fx*P*Fx.transpose() + Fw*Q*Fw.transpose();

//         // get X as the struct state
//         state = vectorToState(X);
//     }

//     EKF::plane EKF::planeCorr() {
//     // Checking if LiDAR point p lies in the corresponding plane, given by
//     normal vector u

//     }

//     void EKF::updateIMU(EKF::State& state, Eigen::VectorXd& IMUState,
//     Eigen::MatrixXd& P) {   // IMUState should be a (24,1) vector
//     // Updates the state vector X and covariance P based on IMU measurement

//         Eigen::VectorXd X(24);
//         X = stateToVector(state);

//         // error
//         Eigen::VectorXd del_X(24);
//         del_X = X - IMUState;

//         // Jacobians
//         Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6,24);
//         H.block<6,6>(0,9) = Eigen::MatrixXd::Identity(6,6);
//         H.block<6,6>(0,18) = Eigen::MatrixXd::Identity(6,6);
//         Eigen::MatrixXd D = Eigen::MatrixXd::Identity(6,6);

//         // noise n = [ng;na] from Rg, Ra
//         Eigen::VectorXd n(6);
//         Eigen::Vector3d ng;
//         double ngx;
//         double ngy;
//         double ngz;
//         Eigen::Vector3d na;
//         double nax;
//         double nay;
//         double naz;

//         ngx = sampleFromGaussian(0, Rg(0,0));
//         ngy = sampleFromGaussian(0, Rg(1,1));
//         ngz = sampleFromGaussian(0, Rg(2,2));
//         ng << ngx, ngy, ngz;

//         nax = sampleFromGaussian(0, Ra(0,0));
//         nay = sampleFromGaussian(0, Ra(1,1));
//         naz = sampleFromGaussian(0, Ra(2,2));
//         na << nax, nay, naz;

//         n << ng, na;

//         // Residual r
//         Eigen::VectorXd r = Eigen::VectorXd::Zero(6, 1);
//         r = H*del_X + D*n;

//         // Kalman gain matrix K
//         Eigen::MatrixXd R(6,6);
//         R.block<3,3>(0,0) = Rg;
//         R.block<3,3>(3,3) = Ra;
//         Eigen::MatrixXd K = P*H.transpose()*(H*P*H.transpose() +
//         R).inverse();

//         // Update state vector X based on the measurement
//         X = X + K*r;
//         state = vectorToState(X);

//         CorrectedIMUState = state; // Ground truth for updateLIDAR

//         // Update covariance matrix P
//         P = (Eigen::MatrixXd::Identity(24,24) - K*H)*P;
//     }

//     void EKF::updateLIDAR(EKF::State& state, EKF::plane plane,
//     Eigen::MatrixXd& P, EKF::State& CorrectedIMUState) {
//     // Updates the state vector X and covariance P based on LiDAR measurement
//         Eigen::VectorXd X(24);
//         X = stateToVector(state);

//         Eigen::Vector3d u;
//         Eigen::Vector3d point;
//         u = plane.u;
//         point = plane.point;  // This is in global frame

//         // LidarState
//         Eigen::VectorXd LidarState = Eigen::VectorXd::Zero(24);
//         Eigen::VectorXd GroundTruth(6);
//         GroundTruth << CorrectedIMUState.Rot, CorrectedIMUState.pos;
//         LidarState.segment<6>(0,0) = GroundTruth;

//         // error
//         Eigen::VectorXd del_X(24);
//         del_X = X - LidarState;

//         // Eigen::Matrix3d Rotation;
//         // Rotation = eulerToRotationMatrix(state.Rot);
//         double yaw = state.Rot(0);
//         double pitch = state.Rot(1);
//         double roll = state.Rot(2);
//         gtsam::Rot3 Rot_object = gtsam::Rot3::RzRyRx(yaw, pitch, roll);
//         // Eigen::Matrix3D Rotation = Rot_object.matrix();

//         // Jacobians
//         Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1,24);
//         Eigen::MatrixXd D(1,3);

//         // defines [p]
//         Eigen::Matrix3d point_skew = skewSymmetric(point);

//         H.block<1,3>(0,0) = -u.transpose()*Rot_object.matrix()*point_skew;
//         H.block<1,3>(0,3) = u.transpose();

//         D = -u.transpose()*Rot_object.matrix();

//         // nl sampled from Rl
//         Eigen::Vector3d nl;
//         double nlx;
//         double nly;
//         double nlz;

//         nlx = sampleFromGaussian(0, Rl(0,0));
//         nly = sampleFromGaussian(0, Rl(1,1));
//         nlz = sampleFromGaussian(0, Rl(2,2));
//         nl << nlx, nly, nlz;

//         // Residual r
//         Eigen::VectorXd r = Eigen::VectorXd::Zero(1, 1);
//         r = H*del_X + D*nl;

//         // Kalman gain matrix K
//         Eigen::MatrixXd R = D*Rl*D.transpose();
//         Eigen::MatrixXd K = P*H.transpose()*(H*P*H.transpose() +
//         R).inverse();

//         // Update state vector X based on the measurement
//         X = X + K*r;
//         state = vectorToState(X);

//         // Update covariance matrix P
//         P = (Eigen::MatrixXd::Identity(P.rows(), P.cols()) - K*H)*P;
//     }

// }

// // NEWER VERSION

// #include "point_lio/point_lio.hpp"

// #include <algorithm>
// #include <execution>

// namespace point_lio {

// double square(const double x) { return x * x; }

// [[nodiscard]] gtsam::Rot3
// compute_plane_R_vec(const Eigen::Vector3d planeNormal,
//                     const Eigen::Vector3d vec) noexcept {
//   const double x_norm = vec.norm();

//   const auto n_dot_vec = planeNormal.dot(vec);
//   const Eigen::Vector3d y =
//       (vec - n_dot_vec * planeNormal).normalized() * x_norm;

//   const double cos_theta = vec.dot(y) / square(x_norm);

//   const double sin_theta = std::sqrt(1.0d - square(cos_theta)) *
//                            (2 * static_cast<double>(n_dot_vec > 0) - 1.0d);

//   const Eigen::Vector3d axis = planeNormal.cross(vec).normalized();

//   return gtsam::Rot3::Quaternion(cos_theta, sin_theta * axis.x(),
//                                  sin_theta * axis.y(), sin_theta * axis.z());
// }

// [[nodiscard]] Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d v) {
//   return (Eigen::Matrix3d() << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(),
//           v.x(), 0)
//       .finished();
// }

// [[nodiscard]] PointLIO::PointLIO() noexcept
//     : m_params{.imuInitializationQuota = 10}, stamp{0.0},
//       world_R_body{gtsam::Rot3::Identity()},
//       world_position{Eigen::Vector3d::Zero()},
//       world_linearVelocity{Eigen::Vector3d::Zero()},
//       world_gravity{0.0, 0.0, -9.81},
//       body_angularVelocity{Eigen::Vector3d::Zero()},
//       body_linearAcceleration{Eigen::Vector3d::Zero()} {

//   // TODO: Take these parameters from OG PointLIO
//   R_imu = Eigen::Matrix<double, 6, 6>::Zero();
//   R_imu(0, 0) = 1e-9;
//   R_imu(1, 1) = 1e-9;
//   R_imu(2, 2) = 1e-9;
//   R_imu(3, 3) = 1e-9;
//   R_imu(4, 4) = 1e-9;
//   R_imu(5, 5) = 1e-9;

//   R_lidar = Eigen::Matrix<double, 3, 3>::Zero();
//   R_lidar(0, 0) = 1e-9;
//   R_lidar(1, 1) = 1e-9;
//   R_lidar(2, 2) = 1e-9;

//   Q = Eigen::Matrix<double, 12, 12>::Zero();
//   Q.block<3, 3>(noise_bias_gyroscope_index, noise_bias_gyroscope_index) =
//       Eigen::Matrix3d::Identity() * 0.00001;
//   Q.block<3, 3>(noise_bias_accelerometer_index,
//                 noise_bias_accelerometer_index) =
//       Eigen::Matrix3d::Identity() * 0.00001;
//   Q.block<3, 3>(noise_gyroscope_index, noise_gyroscope_index) =
//       Eigen::Matrix3d::Identity() * 0.00001;
//   Q.block<3, 3>(noise_accelerometer_index, noise_accelerometer_index) =
//       Eigen::Matrix3d::Identity() * 0.00001;

//   Fw = Eigen::Matrix<double, 24, 12>::Zero();
//   Fw(9, 0) = 1.0;
//   Fw(10, 1) = 1.0;
//   Fw(11, 2) = 1.0;

//   Fw(12, 3) = 1.0;
//   Fw(13, 4) = 1.0;
//   Fw(14, 5) = 1.0;

//   Fw(18, 6) = 1.0;
//   Fw(19, 7) = 1.0;
//   Fw(20, 8) = 1.0;
//   Fw(21, 9) = 1.0;
//   Fw(22, 10) = 1.0;
//   Fw(23, 11) = 1.0;

//   covariance = Eigen::Matrix<double, 24, 24>::Identity();
// }

// void PointLIO::registerImu(const Imu &imu) noexcept {

// //   std::cout << "Entered registerImu" << std::endl;

//   // Estimate IMU biases by assuming the robot is standing still
//   if (!imuBias_gyroscope || !imuBias_accelerometer) {
//     if (m_imuBuffer.empty() || m_imuBuffer.back().stamp < imu.stamp) {
//       m_imuBuffer.push_back(imu);
//       stamp = imu.stamp;
//     }
//     if (m_imuBuffer.size() >= m_params.imuInitializationQuota) {
//       Eigen::Vector3d body_meanMeasuredLinearAcceleration =
//           Eigen::Vector3d::Zero();

//       body_meanMeasuredLinearAcceleration = std::transform_reduce(
//           std::execution::par_unseq, m_imuBuffer.cbegin(), m_imuBuffer.cend(),
//           body_meanMeasuredLinearAcceleration, std::plus<Eigen::Vector3d>(),
//           [this](const auto &imu) {
//             // std::cout << "body_measuredLinearAcceleration: " << imu.body_measuredLinearAcceleration << std::endl;
//             return imu.body_measuredLinearAcceleration / m_imuBuffer.size();
//           });

//       Eigen::Vector3d body_meanMeasuredAngularVelocity =
//           Eigen::Vector3d::Zero();

//       body_meanMeasuredAngularVelocity = std::transform_reduce(
//           std::execution::par_unseq, m_imuBuffer.cbegin(), m_imuBuffer.cend(),
//           body_meanMeasuredAngularVelocity, std::plus<Eigen::Vector3d>(),
//           [this](const auto &imu) {
//             return imu.body_measuredAngularVelocity / m_imuBuffer.size();
//           });

//       const gtsam::Rot3 roll_R_body = compute_plane_R_vec( // Zero yaw
//           {0.0, 1.0, 0.0}, body_meanMeasuredLinearAcceleration);
//       const gtsam::Rot3 world_R_roll = compute_plane_R_vec(
//           {1.0, 0.0, 0.0}, roll_R_body * body_meanMeasuredLinearAcceleration);

//       world_R_body = world_R_roll * roll_R_body;
//       imuBias_gyroscope = body_meanMeasuredAngularVelocity;
//       imuBias_accelerometer = body_meanMeasuredLinearAcceleration +
//                               world_R_body.unrotate(world_gravity);

//     //   std::cout << "imuBias_accelerometer: " << imuBias_accelerometer.value() << std::endl;

//       m_imuBuffer.clear();
//     }
//   } else if (imu.stamp > stamp) {
//     propagateForwardInPlace(imu.stamp - stamp);

//     Eigen::Matrix<double, 6, 24> H;
//     H.block<6, 9>(0, 0).array() = 0.0;
//     H.block<6, 6>(0, 9) = Eigen::Matrix<double, 6, 6>::Identity();
//     H.block<6, 3>(0, 15).array() = 0.0;
//     H.block<6, 6>(0, 18) = Eigen::Matrix<double, 6, 6>::Identity();

//     Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Identity();

//     Eigen::Vector<double, 6> residual;
//     residual.head<3>() = imu.body_measuredAngularVelocity -
//                          body_angularVelocity - imuBias_gyroscope.value();
//     residual.tail<3>() =
//         imu.body_measuredLinearAcceleration - body_linearAcceleration -
//         imuBias_accelerometer.value() + world_R_body.unrotate(world_gravity);

//     std::cout << world_R_body.unrotate(world_gravity) << std::endl;

//     const Eigen::Matrix<double, 24, 6> covariance_H_t =
//         covariance * H.transpose();

//     const Eigen::Matrix<double, 6, 6> S =
//         H * covariance_H_t + D * R_imu * D.transpose();

//     const auto SLLT = S.llt();
//     const Eigen::Vector<double, 24> delta_state =
//         covariance_H_t * SLLT.solve(residual);

//     // Covariance and state update
//     covariance -= covariance_H_t * SLLT.solve(H) * covariance;

//     world_R_body =
//         world_R_body * gtsam::Rot3::Expmap(delta_state.segment<3>(0));
//     world_position += delta_state.segment<3>(3);
//     world_linearVelocity += delta_state.segment<3>(6);
//     imuBias_gyroscope.value() += delta_state.segment<3>(9);
//     imuBias_accelerometer.value() += delta_state.segment<3>(12);
//     body_angularVelocity += delta_state.segment<3>(18);
//     body_linearAcceleration += delta_state.segment<3>(21);

//     // std::cout << "after update: " << world_position << std::endl;

//     // Estimate theta
//     const Eigen::Vector3d theta = delta_state.segment<3>(0);
//     double norm_theta_div_2 = theta.norm() / 2.0;
//     Eigen::Matrix<double, 24, 24> Jt;
//     Jt.block<3, 3>(0, 0) =
//         Eigen::Matrix3d::Identity() - skewSymmetric(theta / 2.0) +
//         (1.0 - norm_theta_div_2 * std::cos(norm_theta_div_2) /
//                    std::sin(norm_theta_div_2)) *
//             skewSymmetric(theta) / theta.norm();
//     Jt.block<3, 21>(0, 3).array() = 0.0;
//     Jt.block<21, 3>(3, 0).array() = 0.0;
//     Jt.block<21, 21>(3, 3) = Eigen::Matrix<double, 21, 21>::Identity();
//     covariance = Jt.transpose() * covariance * Jt;
//     stamp = imu.stamp;
//   }
// }

// void PointLIO::registerPoint(const pcl_types::PointXYZICT &point) noexcept {
//   if (!imuBias_accelerometer || !imuBias_gyroscope) {
//     return;
//   }

//   propagateForwardInPlace(point.timeOffset - stamp);

//   const Eigen::Vector3d world_point =
//       world_R_body * point.getVec3Map().cast<double>() + world_position;

//   const Eigen::Matrix<double, 5, 3> nearest_points =
//       KDT.findNearestNeighbors(world_point);

//   const Eigen::Vector<double, 3> plane_normal = getPlaneNormal(nearest_points);
//   const Eigen::Vector<double, 3> point_in_plane =
//       nearest_points.row(0).transpose();

//   Eigen::Vector<double, 1> hl;
//   hl = -plane_normal.transpose() *
//        (world_R_body * world_point + world_position - point_in_plane);
//   if (abs(hl(0)) > 0.1) {
//     KDT.build2(point.getVec3Map().cast<double>());
//   }

//   Eigen::Matrix<double, 1, 24> H;
//   H.block<1, 18>(0, 6).array() = 0.0;
//   H.block<1, 3>(0, 0) =
//       -plane_normal.transpose() * world_R_body.matrix() *
//       skewSymmetric(Eigen::Vector3d(point.x, point.y, point.z));
//   H.block<1, 3>(0, 3) = plane_normal.transpose();

//   Eigen::MatrixXd D(1, 3);
//   D = -plane_normal.transpose() * world_R_body.matrix();

//   Eigen::Vector<double, 1> residual;
//   residual =
//       -plane_normal.transpose() *
//       (world_R_body.matrix() * Eigen::Vector3d(point.x, point.y, point.z) +
//        world_position - point_in_plane);

//   const Eigen::Matrix<double, 24, 1> covariance_H_t =
//       covariance * H.transpose();

//   const Eigen::Matrix<double, 1, 1> S =
//       H * covariance_H_t + D * R_lidar * D.transpose();

//   const auto SLLT = S.llt();
//   const Eigen::Vector<double, 24> delta_state =
//       covariance_H_t * SLLT.solve(residual);

//   // Covariance and state update
//   covariance -= covariance_H_t * SLLT.solve(H) * covariance;

//   world_R_body = world_R_body * gtsam::Rot3::Expmap(delta_state.segment<3>(0));
//   world_position += delta_state.segment<3>(3);
//   world_linearVelocity += delta_state.segment<3>(6);
//   imuBias_gyroscope.value() += delta_state.segment<3>(9);
//   imuBias_accelerometer.value() += delta_state.segment<3>(12);
//   body_angularVelocity += delta_state.segment<3>(18);
//   body_linearAcceleration += delta_state.segment<3>(21);

//   // Estimate theta
//   const Eigen::Vector3d theta = delta_state.segment<3>(0);
//   double norm_theta_div_2 = theta.norm() / 2.0;
//   Eigen::Matrix<double, 24, 24> Jt;
//   Jt.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() -
//                          skewSymmetric(theta / 2.0) +
//                          (1.0 - norm_theta_div_2 * std::cos(norm_theta_div_2) /
//                                     std::sin(norm_theta_div_2)) *
//                              skewSymmetric(theta) / theta.norm();
//   Jt.block<3, 21>(0, 3).array() = 0.0;
//   Jt.block<21, 3>(3, 0).array() = 0.0;
//   Jt.block<21, 21>(3, 3) = Eigen::Matrix<double, 21, 21>::Identity();
//   covariance = Jt.transpose() * covariance * Jt;
//   stamp = point.timeOffset;
// }

// void PointLIO::propagateForwardInPlace(const double dt) noexcept {
//   const auto I3 = Eigen::Matrix3d::Identity();
//   const auto I3dt = I3 * dt;

//   Eigen::Matrix<double, 24, 24> Fx = Eigen::Matrix<double, 24, 24>();

//   Fx.block<3, 3>(world_R_body_index, world_R_body_index) =
//       gtsam::Rot3::Expmap(-body_angularVelocity * dt).matrix();
//   Fx.block<3, 3>(world_R_body_index, body_angularVelocity_index) = I3dt;

//   Fx.block<3, 3>(world_position_index, world_linearVelocity_index) = I3dt;

//   // TODO
//   Fx.block<3, 3>(world_linearVelocity_index, world_R_body_index) =
//       world_R_body.matrix() * skewSymmetric(body_linearAcceleration) * dt;
//   Fx.block<3, 3>(world_linearVelocity_index, world_gravity_index) = I3dt;

//   Fx.block<3, 3>(world_linearVelocity_index, body_linearAcceleration_index) =
//       world_R_body.matrix() * dt;

//   covariance = Fx * covariance * Fx.transpose() + Fw * Q * Fw.transpose();

// //   std::cout << "before propagate: " << world_position << std::endl;

//   world_R_body = world_R_body * gtsam::Rot3::Expmap(body_angularVelocity * dt);
//   world_position += world_linearVelocity * dt;
//   world_linearVelocity +=
//       (world_R_body * body_linearAcceleration + world_gravity) * dt;


// //   std::cout << "after propagate: " << world_position << std::endl;

// }

// // Eigen::Vector3d computeNormalVector(const Eigen::Vector4d& planeCoeffs) {
// //     Eigen::Vector<double, 3> normalVec;
// //     normalVec << planeCoeffs(0), planeCoeffs(1), planeCoeffs(2);
// //     normalVec.normalize();
// //     return normalVec;
// // }

// Eigen::Vector3d getPlaneNormal(const Eigen::MatrixXd &points) {
//   // Center the points
//   Eigen::Vector3d centroid = points.colwise().mean();
//   Eigen::MatrixXd centered = points.rowwise() - centroid.transpose();

//   // Compute SVD
//   Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
//   Eigen::Vector3d normal = svd.matrixV().col(
//       2); // Right singular vector corresponding to smallest singular value

//   return normal;
// }

// } // namespace point_lio

