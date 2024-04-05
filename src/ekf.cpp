#include "point_lio/ekf.hpp"

namespace point_lio {

    // TODO: initialization of P, X, Q, Ra, Rg, Rl

    
    double sampleFromGaussian(double mean, double stddev) {
    // Function to sample a point from a Gaussian distribution
        
        std::random_device rd;
        std::mt19937 gen(rd()); 
        std::normal_distribution<> dist(mean, stddev); 

        return dist(gen);
    }


    Eigen::VectorXd stateToVector(State& state) {
        Eigen::VectorXd X(24);
        X << state.Rot, state.pos, state.V, state.bg, state.ba, state.g, state.angVel, state.linAcc;
        return X;
    }

    State vectorToState(Eigen::VectorXd& X) {
        // Check if the dimension of X matches the expected state size (24)
        if (X.rows() != 24) {
            throw std::invalid_argument("Input vector X has incorrect dimension");
        }

        State state;
        state.Rot = X.block<3, 3>(0, 0);
        state.pos = X.block<3, 1>(3, 0);
        state.V = X.block<3, 1>(6, 0);
        state.bg = X.block<3, 1>(9, 0);
        state.ba = X.block<3, 1>(12, 0);
        state.g = X.block<3, 1>(15, 0);
        state.angVel = X.block<3, 1>(18, 0);
        state.linAcc = X.block<3, 1>(21, 0);
        return state;
    }



    void predict(State& state, double dt, Eigen::Matrix3d& Rotation, Eigen::MatrixXd& P)  {    //TODO: Remove rotation
    // Predicts the state vector X and covariance P (w/o measurement)

        // Initializing dynamic function f as X_dot
        Eigen::VectorXd X_dot(24);

        // defines [angVel]
        // TODO
        Eigen::Matrix3d angVel_skew = Eigen::Matrix3d::Zero();
        angVel_skew(0, 1) = -state.angVel(1, 0);
        angVel_skew(0, 2) = state.angVel(2, 0); 
        angVel_skew(1, 2) = -state.angVel(2, 1);

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

        // Rotation(k+1) converted from (3,3) to (3,1)
        // to be directly used X(k+1)
        Rotation += (Rotation*angVel_skew)*dt;
        Eigen::Vector3d euler_angles = Rotation.eulerAngles(0, 1, 2);

        // Setting X_dot        
        X_dot.segment<3>(3) = state.V;
        X_dot.segment<3>(6) = Rotation * state.a + state.g;
        X_dot.segment<3>(9) = nbg;
        X_dot.segment<3>(12) = nba;
        X_dot.segment<3>(15) = Eigen::Vector3d::Zero();
        X_dot.segment<3>(18) = wg;
        X_dot.segment<3>(21) = wa;
        
        // Predict the state X(k+1) - discretized model
        Eigen::VectorXd X(24);
        X = stateToVector(state);
        X.segment<21>(3) += dt*X_dot.segment<21>(3);
        X.segment<3>(0) = euler_angles;

        // Prediction matrix Fx and process noise matrix Fw
        Eigen::MatrixXd Fx = Eigen::MatrixXd::Identity(24, 24);
        Eigen::MatrixXd Fw = Eigen::MatrixXd::Zero(24, 12);

        // Setting Fx
        // TODO: define F11, F31, F38
        Fx.block<3,3>(0,0) = exp(-state.angVel*dt);
        Fx.block<3,3>(0,18) = dt*Identity(3,3);
        Fx.block<3,3>(3,6) = dt*Identity(3,3);
        Fx.block<3,3>(6,0) = F31;
        Fx.block<3,3>(6,15) = dt*Identity(3,3);
        Fx.block<3,3>(6,21) = Rotation*dt;

        // Setting Fw
        Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(Q.cols(), Q.cols());
        Fw.block<4, 4>(3, 0) = identity;

        // Update covariance matrix P - 
        P = Fx*P*Fx.transpose() + Fw*Q*Fw.transpose();

        // get X as the struct state
        state = vectorToState(X);
    }



    EKF::Plane EKF::PlaneCorr() {
    // Checking if LiDAR point p lies in the corresponding plane, given by normal vector u

    }



    void updateIMU(State& state, Eigen::VectorXd& IMUState, Eigen::MatrixXd& P) {   // IMUState should be a (24,1) vector
    // Updates the state vector X and covariance P based on IMU measurement

        Eigen::VectorXd X(24);
        X = stateToVector(state);

        // error
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
        P = (Eigen::MatrixXd::Identity(P.rows(), P.cols()) - K*H)*P;
    }



    void updateLIDAR(State& state, plane plane, Eigen::Matrix3d& Rotation, Eigen::MatrixXd& P) {    //TODO: Remove rotation
    // Updates the state vector X and covariance P based on LiDAR measurement

        u = plane.u;
        point = plane.point;

        // Jacobians
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1,24);
        Eigen::MatrixXd D(1,3); 

        // defines [p]
        //TODO
        Eigen::Matrix3d point_skew = Eigen::Matrix3d::Zero();
        point_skew(0, 1) = -point(1, 0);
        point_skew(0, 2) = point(2, 0); 
        point_skew(1, 2) = -point(2, 1);

        H.block<1,3>(0,0) = -u.transpose()*Rotation*point_skew;
        H.block<1,3>(0,3) = u.transpose();

        D = -u.transpose()*Rotation; 

        // nl sampled from Rl
        Eigen::Vector3d nl;
        double nlx;
        double nly;
        double nlz;

        nlx = sampleFromGaussian(0, Rl);
        nly = sampleFromGaussian(0, Rl);
        nlz = sampleFromGaussian(0, Rl); 
        nl << nlx, nly, nlz;

        // Residual r
        Eigen::VectorXd r = Eigen::VectorXd::Zero(1, 1);
        r = H*del_X + D*nl;
        
        // Kalman gain matrix K
        Eigen::MatrixXd K = P*H.transpose()*(H*P*H.transpose() + Rl).inverse();

        // Update state vector X based on the measurement
        X = X + K*r;
        state = vectorToState(X);

        // Update covariance matrix P
        P = (Eigen::MatrixXd::Identity(P.rows(), P.cols()) - K*H)*P;
    }



}
