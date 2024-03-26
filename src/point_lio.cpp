#include "point_lio/point_lio.hpp"

namespace point_lio {

[[nodiscard]] gtsam::NavState PointLIO::registerImu(const Imu &imu) noexcept {}

[[nodiscard]] std::vector<pcl_types::PointXYZICT>
PointLIO::registerScan(const pcl_types::LidarScanStamped &scan) noexcept {}
} // namespace point_lio
