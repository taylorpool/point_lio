#pragma once

#include "velodyne_decoder/velodyne_decoder.hpp"

#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodyneScan.h>

#include <sensor_msgs/PointCloud2.h>

namespace velodyne_decoder {

class VLP16Decoder {
private:
  constexpr static size_t kNUM_CHANNELS = 16;
  constexpr static float kMILLI_TO_UNIT = 0.001;
  std::array<float, kNUM_CHANNELS> m_channelToSinVerticalAngle;
  std::array<float, kNUM_CHANNELS> m_channelToCosVerticalAngle;

  float m_azimuth;

public:
  VLP16Decoder();

  pcl_types::LidarScanStamped
  decode(const velodyne_msgs::VelodyneScan::ConstPtr &scan);
};

class VLP32CDecoder {
private:
  constexpr static size_t kNUM_CHANNELS = 32;
  std::array<float, kNUM_CHANNELS> m_channelToSinVerticalAngle;
  std::array<float, kNUM_CHANNELS> m_channelToCosVerticalAngle;
  float m_azimuth;

public:
  VLP32CDecoder();

  pcl_types::LidarScanStamped
  decode(const velodyne_msgs::VelodyneScan::ConstPtr &scan);
};

} // namespace velodyne_decoder
