#include "point_lio/point_lio.hpp"

namespace point_lio {

[[nodiscard]] PointLIO::PointLIO() noexcept :
m_imuInitializationQuota(10),
m_preintegrationParams(boost::make_shared<gtsam::PreintegrationCombinedParams>(Eigen::Vector3d(0.0, 0.0, -9.81))),
m_imuState(gtsam::Pose3::Identity(), Eigen::Vector3d::Zero()),
m_imuBias(){
        imu_state.Rot.setZero();
        imu_state.pos.setZero();
        imu_state.V.setZero();
        imu_state.bg.setZero();
        imu_state.ba.setZero();
        imu_state.g << 0.0, 0.0, -9.81; 
        imu_state.angVel.setZero();
        imu_state.linAcc.setZero();
}

[[nodiscard]] gtsam::NavState PointLIO::registerImu(Imu_State& imu_state, const Imu &imu) noexcept {

    if(m_imuBias && imu.stamp > m_currentStamp)
    {
        const Eigen::Vector3d body_acceleration = imu.body_measuredLinearAcceleration 
        - m_imuBias->accelerometer()
        + m_imuState.pose().rotation().unrotate(m_preintegrationParams->getGravity());
        const Eigen::Vector3d body_angularVelocity = imu.body_measuredAngularVelocity - m_imuBias->gyroscope();
        const double dt = imu.stamp - m_currentStamp;
        
        m_imuState = m_imuState.update(body_acceleration, body_angularVelocity, dt, boost::none, boost::none, boost::none);
        m_currentStamp = imu.stamp;

        delt = dt;

        // Getting state estimate based on IMU read
        Eigen::Vector3d pos = m_imuState.pose().translation();
        Eigen::Vector3d vel = m_imuState.velocity();
        gtsam::Rot3 rotMat = m_imuState.pose().rotation();

        imu_state.Rot = rotMat.rpy();
        imu_state.pos = pos;
        imu_state.V = vel;
        imu_state.bg = m_imuBias->gyroscope();
        imu_state.ba = m_imuBias->accelerometer();

        imu_state.angVel = body_angularVelocity;
        imu_state.linAcc = body_acceleration;
    }
    else
    {
        m_imuBuffer.push_back(imu);
        if(std::ranges::size(m_imuBuffer) > m_imuInitializationQuota)
        {
            m_currentStamp = imu.stamp;
            m_imuBias = gtsam::imuBias::ConstantBias(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
            m_imuBuffer.clear();
        }
    }

    return m_imuState;
}

[[nodiscard]] std::vector<pcl_types::PointXYZICT>
PointLIO::registerScan(const pcl_types::LidarScanStamped &scan) noexcept {}

nav_msgs::Odometry PointLIO::NavstateToOdometry(gtsam::NavState odometry){
    nav_msgs::Odometry state;

    Eigen::Vector3d pos = odometry.pose().translation();
    Eigen::Vector3d vel = odometry.velocity();
    gtsam::Rot3 rotMat = odometry.pose().rotation();
    gtsam::Quaternion quat = rotMat.toQuaternion();

    state.pose.pose.position.x = pos.x();
    state.pose.pose.position.y = pos.y();
    state.pose.pose.position.z = pos.z();

    state.twist.twist.linear.x = vel.x();
    state.twist.twist.linear.y = vel.y();
    state.twist.twist.linear.z = vel.z();

    state.pose.pose.orientation.w = quat.w();
    state.pose.pose.orientation.x = quat.x();
    state.pose.pose.orientation.y = quat.y();
    state.pose.pose.orientation.z = quat.z();

    return state;
}

// void getImu_State(Imu_State& imu_state, const Imu &imu){

//     m_imuState = PointLIO::registerImu(imu);

//     Eigen::Vector3d pos = odometry.pose().translation();
//     Eigen::Vector3d vel = odometry.velocity();
//     gtsam::Rot3 rotMat = odometry.pose().rotation();

//     imu_state.Rot = rotMat.rpy();
//     imu_state.pos = pos;
//     imu_state.V = vel;
//     imu_state.bg = m_imuBias->gyroscope();
//     imu_state.ba = m_imuBias->accelerometer();
    
// }

} // namespace point_lio
