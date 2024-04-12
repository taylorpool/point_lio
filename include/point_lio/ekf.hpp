// #ifndef POINT_LIO_EKF_HPP
// #define POINT_LIO_EKF_HPP

// #include <Eigen/Dense>
// #include <cmath>
// #include <gtsam/geometry/Rot3.h>
// #include <random>

// namespace point_lio {

// class EKF {
// public:
//   // Setting mean and stddev
//   double mean;
//   double stddev;

//   struct State {
//     Eigen::Vector3d Rot;    // angles (rotation)
//     Eigen::Vector3d pos;    // position vector
//     Eigen::Vector3d V;      // velocity vector
//     Eigen::Vector3d bg;     // gyro bias vector
//     Eigen::Vector3d ba;     // accelerometer bias vector
//     Eigen::Vector3d g;      // gravity vector
//     Eigen::Vector3d angVel; // angular velocity vector
//     Eigen::Vector3d linAcc; // linear acceleration vector
//   };

//   struct plane {
//     Eigen::VectorXd point;
//     Eigen::VectorXd u;
//   };

//   State state;

//   State CorrectedIMUState; // Ground truth for updateLIDAR

//   // Constructor to initialize P and X
//   EKF();

//   // Sampling a point from a Gaussian
//   double sampleFromGaussian(double mean, double stddev);

//   // Vector to Skew Symm
//   Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v);

//   // vector to rotation matrix
//   // Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d& euler);

//   // getting vector X from the State struct
//   Eigen::VectorXd stateToVector(State &state);

//   // getting struct State from vector X
//   State vectorToState(Eigen::VectorXd &X);

//   // Predict step
//   void predict(State &state, double dt, Eigen::MatrixXd &P);

//   // Checking plane correspondance
//   plane planeCorr();

//   // Update step
//   void updateIMU(State &state, Eigen::VectorXd &IMUState, Eigen::MatrixXd
//   &P); void updateLIDAR(State &state, plane plane, Eigen::MatrixXd &P,
//                    State &CorrectedIMUState);
// };
// } // namespace point_lio

// #endif
