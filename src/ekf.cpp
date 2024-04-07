#include "point_lio/ekf.hpp"

namespace point_lio {

    // TODO: initialization of P, X, Q, Ra, Rg, Rl
    EKF::EKF() {
        P.setZero(); 
        state.Rot.setZero();
        state.pos.setZero();
        state.V.setZero();
        state.bg.setZero();
        state.ba.setZero();
        state.g << 0.0, 0.0, -9.81; 
        state.angVel.setZero();
        state.linAcc.setZero();

        gen.seed(0);
    };

    
    double EKF::sampleFromGaussian(double mean, double stddev) { // to define this only once 
    // Function to sample a point from a Gaussian distribution
        
        std::normal_distribution<double> dist(mean, stddev); 

        return dist(gen);
    }

    Eigen::Matrix3d EKF::skewSymmetric(const Eigen::Vector3d& v) {
        Eigen::Matrix3d skew;
        skew <<  0, -v[2],  v[1],
                v[2],  0,  -v[0],
                -v[1],  v[0], 0;
        return skew;
    }


    // Eigen::Matrix3d EKF::eulerToRotationMatrix(Eigen::Vector3d& euler) {
    //     double roll = euler(0);
    //     double pitch = euler(1);
    //     double yaw = euler(2);

    //     Eigen::Matrix3d rotationMatrix;
    //     rotationMatrix << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll),
    //                     sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll),
    //                     -sin(pitch),          cos(pitch)*sin(roll),                                       cos(pitch)*cos(roll);
    //     return rotationMatrix;
    // }


    Eigen::VectorXd EKF::stateToVector(EKF::State& state) {
        Eigen::VectorXd X(24);
        X << state.Rot, state.pos, state.V, state.bg, state.ba, state.g, state.angVel, state.linAcc;
        return X;
    }

    EKF::State EKF::vectorToState(Eigen::VectorXd& X) {
     
        State state;
        state.Rot = X.block<3, 1>(0, 0);
        state.pos = X.block<3, 1>(3, 0);
        state.V = X.block<3, 1>(6, 0);
        state.bg = X.block<3, 1>(9, 0);
        state.ba = X.block<3, 1>(12, 0);
        state.g = X.block<3, 1>(15, 0);
        state.angVel = X.block<3, 1>(18, 0);
        state.linAcc = X.block<3, 1>(21, 0);
        return state;
    }



    void EKF::predict(EKF::State& state, double dt, Eigen::MatrixXd& P)  {    
    // Predicts the state vector X and covariance P (w/o measurement)

        // Initializing dynamic function f as X_dot
        Eigen::VectorXd X_dot(24);

        // defines [angVel]
        Eigen::Matrix3d angVel_skew = skewSymmetric(state.angVel);

        // defines noises and biases
        Eigen::Vector3d nbg;
        double nbgx; 
        double nbgy;
        double nbgz;
        Eigen::Vector3d nba;
        double nbax;
        double nbay;
        double nbaz;
        Eigen::Vector3d wg;
        double wgx;
        double wgy;
        double wgz;
        Eigen::Vector3d wa;
        double wax;
        double way;
        double waz;

        Eigen::Matrix3d Qbg = Q.block<3,3>(0,0);
        Eigen::Matrix3d Qba = Q.block<3,3>(3,3);
        Eigen::Matrix3d Qg = Q.block<3,3>(6,6);
        Eigen::Matrix3d Qa = Q.block<3,3>(9,9);

        // Assuming uncorrelated?
        nbgx = sampleFromGaussian(0, Qbg(0,0));
        nbgy = sampleFromGaussian(0, Qbg(1,1));
        nbgz = sampleFromGaussian(0, Qbg(2,2)); 
        nbg << nbgx, nbgy, nbgz;

        nbax = sampleFromGaussian(0, Qba(0,0));
        nbay = sampleFromGaussian(0, Qba(1,1));
        nbaz = sampleFromGaussian(0, Qba(2,2));
        nba << nbax, nbay, nbaz;

        wgx = sampleFromGaussian(0, Qg(0,0));
        wgy = sampleFromGaussian(0, Qg(1,1));
        wgz = sampleFromGaussian(0, Qg(2,2));
        wg << wgx, wgy, wgz;

        wax = sampleFromGaussian(0, Qa(0,0));
        way = sampleFromGaussian(0, Qa(1,1));
        waz = sampleFromGaussian(0, Qa(2,2));
        wa << wax, way, waz;

        // Rotation(k+1) converted from (3,1) to (3,1) back to (3,1)
        // to be directly used X(k+1)
        // Eigen::Matrix3d Rotation; 
        // Rotation = eulerToRotationMatrix(state.Rot);
        // Rotation += (Rotation*angVel_skew)*dt;
        // Eigen::Vector3d euler_angles = Rotation.eulerAngles(0, 1, 2);  // remove euler angles
        double yaw = state.Rot(0);
        double pitch = state.Rot(1);
        double roll = state.Rot(2);
        gtsam::Rot3 Rot_object = gtsam::Rot3::RzRyRx(yaw, pitch, roll);
        // Eigen::Matrix3d Rotation = Rot_object.matrix();
    

        // Setting X_dot    
        X_dot.segment<3>(3) = state.V;
        X_dot.segment<3>(6) = Rot_object.matrix()*state.linAcc + state.g;
        X_dot.segment<3>(9) = nbg;
        X_dot.segment<3>(12) = nba;
        X_dot.segment<3>(15) = Eigen::Vector3d::Zero();
        X_dot.segment<3>(18) = wg;
        X_dot.segment<3>(21) = wa;
        
        // Rotation at next step
        Rot_object.matrix() += (Rot_object.matrix()*angVel_skew)*dt;
        gtsam::Vector3 euler_angles = Rot_object.rpy();

        // Predict the state X(k+1) - discretized model
        Eigen::VectorXd X(24);
        X = stateToVector(state);
        X.segment<21>(3) += dt*X_dot.segment<21>(3);
        X.segment<3>(0) = euler_angles;

        // Prediction matrix Fx and process noise matrix Fw
        Eigen::MatrixXd Fx = Eigen::MatrixXd::Identity(24, 24);
        Eigen::MatrixXd Fw = Eigen::MatrixXd::Zero(24, 12);

        // Setting Fx
        // TODO: define F31
        const gtsam::Rot3 exp = gtsam::Rot3::Expmap(state.angVel*dt);

        Fx.block<3,3>(0,0) = exp.matrix();
        Fx.block<3,3>(0,18) = dt*Eigen::MatrixXd::Identity(3,3);
        Fx.block<3,3>(3,6) = dt*Eigen::MatrixXd::Identity(3,3);
        //Fx.block<3,3>(6,0) = F31;
        Fx.block<3,3>(6,15) = dt*Eigen::MatrixXd::Identity(3,3);
        Fx.block<3,3>(6,21) = Rot_object.matrix()*dt;

        // Setting Fw
        Fw.block<6, 6>(9, 0) = Eigen::MatrixXd::Identity(6, 6);
        Fw.block<6, 6>(18, 6) = Eigen::MatrixXd::Identity(6, 6);

        // Update covariance matrix P - 
        P = Fx*P*Fx.transpose() + Fw*Q*Fw.transpose();

        // get X as the struct state
        state = vectorToState(X);
    }



    EKF::plane EKF::planeCorr() {
    // Checking if LiDAR point p lies in the corresponding plane, given by normal vector u

    }



    void EKF::updateIMU(EKF::State& state, Eigen::VectorXd& IMUState, Eigen::MatrixXd& P) {   // IMUState should be a (24,1) vector
    // Updates the state vector X and covariance P based on IMU measurement

        Eigen::VectorXd X(24);
        X = stateToVector(state);

        // error
        Eigen::VectorXd del_X(24);
        del_X = X - IMUState;

        // Jacobians
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6,24);
        H.block<6,6>(0,9) = Eigen::MatrixXd::Identity(6,6);
        H.block<6,6>(0,18) = Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd D = Eigen::MatrixXd::Identity(6,6);   

        // noise n = [ng;na] from Rg, Ra
        Eigen::VectorXd n(6); 
        Eigen::Vector3d ng;
        double ngx; 
        double ngy;
        double ngz;
        Eigen::Vector3d na;
        double nax;
        double nay;
        double naz;

        ngx = sampleFromGaussian(0, Rg(0,0));
        ngy = sampleFromGaussian(0, Rg(1,1));
        ngz = sampleFromGaussian(0, Rg(2,2)); 
        ng << ngx, ngy, ngz;

        nax = sampleFromGaussian(0, Ra(0,0));
        nay = sampleFromGaussian(0, Ra(1,1));
        naz = sampleFromGaussian(0, Ra(2,2));
        na << nax, nay, naz;

        n << ng, na; 

        // Residual r
        Eigen::VectorXd r = Eigen::VectorXd::Zero(6, 1);
        r = H*del_X + D*n;

        // Kalman gain matrix K
        Eigen::MatrixXd R(6,6); 
        R.block<3,3>(0,0) = Rg;
        R.block<3,3>(3,3) = Ra; 
        Eigen::MatrixXd K = P*H.transpose()*(H*P*H.transpose() + R).inverse();

        // Update state vector X based on the measurement
        X = X + K*r;
        state = vectorToState(X);

        // Update covariance matrix P
        P = (Eigen::MatrixXd::Identity(24,24) - K*H)*P;
    }



    void EKF::updateLIDAR(EKF::State& state, EKF::plane plane, Eigen::MatrixXd& P, EKF::State& CorrectedIMUState) {   
    // Updates the state vector X and covariance P based on LiDAR measurement
        Eigen::VectorXd X(24);
        X = stateToVector(state);

        Eigen::Vector3d u;
        Eigen::Vector3d point;
        u = plane.u;
        point = plane.point;  // This is in global frame

        // LidarState 
        Eigen::VectorXd LidarState = Eigen::VectorXd::Zero(24);
        Eigen::VectorXd GroundTruth(6); 
        GroundTruth << CorrectedIMUState.Rot, CorrectedIMUState.pos; 
        LidarState.segment<6>(0,0) = GroundTruth; 

        // error
        Eigen::VectorXd del_X(24);
        del_X = X - LidarState;

        // Eigen::Matrix3d Rotation; 
        // Rotation = eulerToRotationMatrix(state.Rot);
        double yaw = state.Rot(0);
        double pitch = state.Rot(1);
        double roll = state.Rot(2);
        gtsam::Rot3 Rot_object = gtsam::Rot3::RzRyRx(yaw, pitch, roll);
        // Eigen::Matrix3D Rotation = Rot_object.matrix();
    

        // Jacobians
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1,24);
        Eigen::MatrixXd D(1,3); 

        // defines [p]
        Eigen::Matrix3d point_skew = skewSymmetric(point);

        H.block<1,3>(0,0) = -u.transpose()*Rot_object.matrix()*point_skew;
        H.block<1,3>(0,3) = u.transpose();

        D = -u.transpose()*Rot_object.matrix(); 

        // nl sampled from Rl
        Eigen::Vector3d nl;
        double nlx;
        double nly;
        double nlz;

        nlx = sampleFromGaussian(0, Rl(0,0));
        nly = sampleFromGaussian(0, Rl(1,1));
        nlz = sampleFromGaussian(0, Rl(2,2)); 
        nl << nlx, nly, nlz;

        // Residual r
        Eigen::VectorXd r = Eigen::VectorXd::Zero(1, 1);
        r = H*del_X + D*nl;
        
        // Kalman gain matrix K
        Eigen::MatrixXd R = D*Rl*D.transpose();
        Eigen::MatrixXd K = P*H.transpose()*(H*P*H.transpose() + R).inverse();

        // Update state vector X based on the measurement
        X = X + K*r;
        state = vectorToState(X);

        // Update covariance matrix P
        P = (Eigen::MatrixXd::Identity(P.rows(), P.cols()) - K*H)*P;
    }



}
