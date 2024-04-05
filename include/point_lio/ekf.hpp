#ifndef POINT_LIO_EKF_HPP
#define POINT_LIO_EKF_HPP

#include <Eigen/Dense>
#include <cmath>


namespace point_lio {


class EKF{
public:
    Eigen::MatrixXd P; // Initial covariance matrix - (24,24)
    static const Eigen::MatrixXd Q; // Process noise - diag(12,12)
    static const Eigen::MatrixXd Ra; // Accelerometer Measurement Noise - diag(3,3)
    static const Eigen::MatrixXd Rg; // Gyro Measurement Noise - diag(3,3)
    static const double Rl; // LiDAR Measurement Noise 

    // Setting mean and stddev
    double mean;
    double stddev;

    struct State {
        Eigen::Vector3d Rot; // Euler angles (rotation)
        Eigen::Vector3d pos; // position vector
        Eigen::Vector3d V; // velocity vector
        Eigen::Vector3d bg; // gyro bias vector
        Eigen::Vector3d ba; // accelerometer bias vector
        Eigen::Vector3d g; // gravity vector
        Eigen::Vector3d angVel; // angular velocity vector
        Eigen::Vector3d linAcc; // linear acceleration vector
    }
    
    struct plane {
        Eigen::VectorXd point;
        Eigen::VectorXd u;
    };


    // Constructor to initialize P and X
    EKF() {
        P.setZero(); 
        state.Rot.setZero();
        state.pos.setZero();
        state.V.setZero();
        state.bg.setZero();
        state.ba.setZero();
        state.g << 0.0, 0.0, -9.81; 
        state.angVel.setZero();
        state.linAcc.setZero();
    };

    // Sampling a point from a Gaussian
    double sampleFromGaussian(double mean, double stddev);

    // getting vector X from the State struct
    Eigen::VectorXd stateToVector(State& state);

    // getting struct State from vector X
    State vectorToState(Eigen::VectorXd& X); 

    // Predict step 
    void predict(State& state, double dt, Eigen::Matrix3d& Rotation, Eigen::MatrixXd& P)  {    
    
    // Checking plane correspondance
    plane planeCorr();

    // Update step 
    void updateIMU(State& state, Eigen::VectorXd& IMUState, Eigen::MatrixXd& P);
    void updateLIDAR(State& state, plane& plane, Eigen::MatrixXd& P) ;

};

}
}
#endif
