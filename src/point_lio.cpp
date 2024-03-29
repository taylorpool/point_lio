#include "point_lio/point_lio.hpp"

namespace point_lio {

    [[nodiscard]] PointLIO::PointLIO() noexcept :
    m_imuInitializationQuota(10),
    m_preintegrationParams(boost::make_shared<gtsam::PreintegrationCombinedParams>(Eigen::Vector3d(0.0, 0.0, -9.81))),
    m_imuState(gtsam::Pose3::Identity(), Eigen::Vector3d::Zero()),
    m_imuBias() {

    }

[[nodiscard]] gtsam::NavState PointLIO::registerImu(const Imu &imu) noexcept {

    if(m_imuBias && imu.stamp > m_currentStamp)
    {
        const Eigen::Vector3d body_acceleration = imu.body_measuredLinearAcceleration 
        - m_imuBias->accelerometer()
        + m_imuState.pose().rotation().unrotate(m_preintegrationParams->getGravity());
        const Eigen::Vector3d body_angularVelocity = imu.body_measuredAngularVelocity - m_imuBias->gyroscope();
        const double dt = imu.stamp - m_currentStamp;
        
        m_imuState = m_imuState.update(body_acceleration, body_angularVelocity, dt, boost::none, boost::none, boost::none);
        m_currentStamp = imu.stamp;
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
} // namespace point_lio
