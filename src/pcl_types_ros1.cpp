#include "pcl_types/pcl_types_ros1.hpp"

#include <span>

namespace pcl_types::ros1 {

[[nodiscard]] bool fromMsg(const sensor_msgs::PointCloud2 &msg,
                           pcl_types::LidarScanStamped &scan) noexcept {
  scan.stamp = msg.header.stamp.toSec();
  scan.cloud.clear();
  scan.cloud.reserve(msg.width);

  for (size_t pointCount = static_cast<size_t>(0); pointCount < msg.width;
       ++pointCount) {

    pcl_types::PointXYZICT point;
    const size_t pointIndex = pointCount * msg.point_step;
    point.x = *reinterpret_cast<const decltype(point.x) *>(
        msg.data.data() + pointIndex + pcl_types::PointXYZICT::kX_INDEX);
    point.y = *reinterpret_cast<const decltype(point.y) *>(
        msg.data.data() + pointIndex + pcl_types::PointXYZICT::kY_INDEX);
    point.z = *reinterpret_cast<const decltype(point.z) *>(
        msg.data.data() + pointIndex + pcl_types::PointXYZICT::kZ_INDEX);

    if (Eigen::Map<Eigen::Vector3f>(&point.x).allFinite()) {
      point.intensity = *reinterpret_cast<const decltype(point.intensity) *>(
          msg.data.data() + pointIndex +
          pcl_types::PointXYZICT::kINTENSITY_INDEX);
      point.channel = *reinterpret_cast<const decltype(point.channel) *>(
          msg.data.data() + pointIndex +
          pcl_types::PointXYZICT::kCHANNEL_INDEX);
      point.timeOffset = *reinterpret_cast<const decltype(point.timeOffset) *>(
          msg.data.data() + pointIndex +
          pcl_types::PointXYZICT::kTIME_OFFSET_INDEX);
      scan.cloud.push_back(point);
    }
  }
  return true;
}

[[nodiscard]] bool toMsg(const std::vector<pcl_types::PointXYZICT> &cloud,
                         sensor_msgs::PointCloud2 &msg) noexcept {
  msg.height = 1;

  msg.width = cloud.size();

  msg.fields.resize(6);
  msg.fields[0].name = "x";
  msg.fields[0].offset = pcl_types::PointXYZICT::kX_INDEX;
  msg.fields[0].datatype = getPointField<decltype(pcl_types::PointXYZICT::x)>();
  msg.fields[0].count = 1;

  msg.fields[1].name = "y";
  msg.fields[1].offset = pcl_types::PointXYZICT::kY_INDEX;
  msg.fields[1].datatype = getPointField<decltype(pcl_types::PointXYZICT::y)>();
  msg.fields[1].count = 1;

  msg.fields[2].name = "z";
  msg.fields[2].offset = pcl_types::PointXYZICT::kZ_INDEX;
  msg.fields[2].datatype = getPointField<decltype(pcl_types::PointXYZICT::z)>();
  msg.fields[2].count = 1;

  msg.fields[3].name = "intensity";
  msg.fields[3].offset = pcl_types::PointXYZICT::kINTENSITY_INDEX;
  msg.fields[3].datatype =
      getPointField<decltype(pcl_types::PointXYZICT::intensity)>();
  msg.fields[3].count = 1;

  msg.fields[4].name = "channel";
  msg.fields[4].offset = pcl_types::PointXYZICT::kCHANNEL_INDEX;
  msg.fields[4].datatype =
      getPointField<decltype(pcl_types::PointXYZICT::channel)>();
  msg.fields[4].count = 1;

  msg.fields[5].name = "timeOffset";
  msg.fields[5].offset = pcl_types::PointXYZICT::kTIME_OFFSET_INDEX;
  msg.fields[5].datatype =
      getPointField<decltype(pcl_types::PointXYZICT::timeOffset)>();
  msg.fields[5].count = 1;

  msg.is_bigendian = (std::endian::native == std::endian::big);

  msg.point_step = PointXYZICT::kSIZE;

  msg.row_step = msg.width * msg.point_step;

  msg.data.clear();
  msg.data.reserve(msg.row_step * msg.height);
  for (const auto &point : cloud) {
    serialize(msg.data, point);
  }
  msg.is_dense = false;
  return true;
}

[[nodiscard]] bool toMsg(const pcl_types::LidarScanStamped &scan,
                         sensor_msgs::PointCloud2 &msg) noexcept {
  msg.header.stamp.fromSec(scan.stamp);
  return toMsg(scan.cloud, msg);
}

} // namespace pcl_types::ros1
