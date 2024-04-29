#include "velodyne_decoder/velodyne_decoder_ros.hpp"

#include <ranges>
#include <span>

namespace velodyne_decoder {

VLP16Decoder::VLP16Decoder() : m_azimuth(-1.0) {
  constexpr std::array<float, kNUM_CHANNELS> channelToVerticalAngle{
      -15.0f * kDEG_TO_RAD, 1.0f * kDEG_TO_RAD,   -13.0f * kDEG_TO_RAD,
      3.0f * kDEG_TO_RAD,   -11.0f * kDEG_TO_RAD, 5.0f * kDEG_TO_RAD,
      -9.0f * kDEG_TO_RAD,  7.0f * kDEG_TO_RAD,   -7.0f * kDEG_TO_RAD,
      9.0f * kDEG_TO_RAD,   -5.0f * kDEG_TO_RAD,  11.0f * kDEG_TO_RAD,
      -3.0f * kDEG_TO_RAD,  13.0f * kDEG_TO_RAD,  -1.0f * kDEG_TO_RAD,
      15.0f * kDEG_TO_RAD};

  std::ranges::transform(channelToVerticalAngle,
                         m_channelToSinVerticalAngle.begin(),
                         [](float x) { return std::sin(x); });
  std::ranges::transform(channelToVerticalAngle,
                         m_channelToCosVerticalAngle.begin(),
                         [](float x) { return std::cos(x); });
}

pcl_types::LidarScanStamped
VLP16Decoder::decode(const velodyne_msgs::VelodyneScan::ConstPtr &scan) {
  pcl_types::LidarScanStamped result;
  result.stamp = scan->header.stamp.toSec();
  for (const auto &packet : scan->packets) {
    const double packetTimeOffset = packet.stamp.toSec() - result.stamp;

    constexpr std::array<float, kNUM_CHANNELS> channelToVerticalCorrection{
        11.2f * kMILLI_TO_UNIT, -0.7f * kMILLI_TO_UNIT, 9.7f * kMILLI_TO_UNIT,
        -2.2f * kMILLI_TO_UNIT, 8.1f * kMILLI_TO_UNIT,  -3.7f * kMILLI_TO_UNIT,
        6.6f * kMILLI_TO_UNIT,  -5.1f * kMILLI_TO_UNIT, 5.1f * kMILLI_TO_UNIT,
        -6.6f * kMILLI_TO_UNIT, 3.7f * kMILLI_TO_UNIT,  -8.1f * kMILLI_TO_UNIT,
        2.2f * kMILLI_TO_UNIT,  -9.7f * kMILLI_TO_UNIT, 0.7f * kMILLI_TO_UNIT,
        -11.2f * kMILLI_TO_UNIT};
    constexpr size_t kNUM_BLOCKS = 12;
    constexpr size_t kSEQUENCES_PER_BLOCK = 2;
    constexpr size_t kFLAG_SIZE = 2;
    constexpr size_t kAZIMUTH_SIZE = 2;
    constexpr size_t kRANGE_SIZE = 2;
    constexpr size_t kINTENSITY_SIZE = 1;
    constexpr size_t kBLOCK_SIZE =
        kFLAG_SIZE + kAZIMUTH_SIZE +
        kNUM_CHANNELS * kSEQUENCES_PER_BLOCK * (kRANGE_SIZE + kINTENSITY_SIZE);

    constexpr float kCHANNEL_TIME = 2.304e-6f;
    constexpr float kRECHARGE_TIME = 18.43e-6f;
    constexpr float kSEQUENCE_TIME =
        kCHANNEL_TIME * kNUM_CHANNELS + kRECHARGE_TIME;
    constexpr float kBLOCK_TIME = kSEQUENCE_TIME * kSEQUENCES_PER_BLOCK;

    size_t index = 0;
    for (size_t block = 0; block < kNUM_BLOCKS; ++block) {
      index += kFLAG_SIZE;
      constexpr float kCENTI_TO_UNIT = 0.01;
      const float blockAzimuth =
          static_cast<float>(getBytes<kAZIMUTH_SIZE>(&packet.data[index])) *
              kCENTI_TO_UNIT * kDEG_TO_RAD +
          std::numbers::pi_v<float> / 2.0f;

      float azimuthRate = 0.0;
      if (m_azimuth >= 0.0) {
        azimuthRate = (blockAzimuth - m_azimuth);
        if (azimuthRate < 0) {
          constexpr float kTWO_PI = std::numbers::pi_v<float> * 2;
          azimuthRate += kTWO_PI;
        }
        azimuthRate /= kSEQUENCE_TIME;
      }

      index += kAZIMUTH_SIZE;
      for (size_t sequence = 0; sequence < kSEQUENCES_PER_BLOCK; ++sequence) {
        for (size_t channel = 0; channel < kNUM_CHANNELS; ++channel) {
          const float range =
              static_cast<float>(static_cast<uint16_t>(2) *
                                 getBytes<kRANGE_SIZE>(&packet.data[index])) *
              kMILLI_TO_UNIT;
          index += kRANGE_SIZE;
          const uint8_t intensity =
              getBytes<kINTENSITY_SIZE>(&packet.data[index]);
          index += kINTENSITY_SIZE;
          const float timeOffset =
              packetTimeOffset + kBLOCK_TIME * static_cast<float>(block) +
              kSEQUENCE_TIME * static_cast<float>(sequence) +
              kCHANNEL_TIME * static_cast<float>(channel);
          const float preciseAzimuth =
              blockAzimuth +
              azimuthRate * (kCHANNEL_TIME * static_cast<float>(channel) +
                             kSEQUENCE_TIME * sequence);

          const float xyRange = range * m_channelToCosVerticalAngle[channel];
          result.cloud.push_back(
              {.x = xyRange * std::sin(preciseAzimuth),
               .y = xyRange * std::cos(preciseAzimuth),
               .z = range * m_channelToSinVerticalAngle[channel] +
                    channelToVerticalCorrection[channel],
               .intensity = intensity,
               .channel = static_cast<uint8_t>(channel),
               .timeOffset = timeOffset});
          if (!result.cloud.back().getVector3fMap().allFinite() ||
              result.cloud.back().getVector3fMap().squaredNorm() < 0.05) {
            result.cloud.pop_back();
          }
        }
      }
    }
  }
  return result;
}

VLP32CDecoder::VLP32CDecoder() : m_azimuth(-1.0) {
  constexpr std::array<float, kNUM_CHANNELS> channelToVerticalAngle{
      -25.0f * kDEG_TO_RAD,   -1.0f * kDEG_TO_RAD,   -1.667f * kDEG_TO_RAD,
      -15.639f * kDEG_TO_RAD, -11.31f * kDEG_TO_RAD, 0.0f * kDEG_TO_RAD,
      -0.667f * kDEG_TO_RAD,  -8.843f * kDEG_TO_RAD, -7.254f * kDEG_TO_RAD,
      0.333f * kDEG_TO_RAD,   -0.333f * kDEG_TO_RAD, -6.148f * kDEG_TO_RAD,
      -5.333f * kDEG_TO_RAD,  1.333f * kDEG_TO_RAD,  0.667f * kDEG_TO_RAD,
      -4.0f * kDEG_TO_RAD,    -4.667f * kDEG_TO_RAD, 1.667f * kDEG_TO_RAD,
      1.0f * kDEG_TO_RAD,     -3.667f * kDEG_TO_RAD, -3.333f * kDEG_TO_RAD,
      3.333f * kDEG_TO_RAD,   2.333f * kDEG_TO_RAD,  -2.667f * kDEG_TO_RAD,
      -3.0f * kDEG_TO_RAD,    7.0f * kDEG_TO_RAD,    4.667f * kDEG_TO_RAD,
      -2.333f * kDEG_TO_RAD,  -2.0f * kDEG_TO_RAD,   15.0f * kDEG_TO_RAD,
      10.333f * kDEG_TO_RAD,  -1.333f * kDEG_TO_RAD};

  std::ranges::transform(channelToVerticalAngle,
                         m_channelToSinVerticalAngle.begin(),
                         [](float x) { return std::sin(x); });
  std::ranges::transform(channelToVerticalAngle,
                         m_channelToCosVerticalAngle.begin(),
                         [](float x) { return std::cos(x); });
}

pcl_types::LidarScanStamped
VLP32CDecoder::decode(const velodyne_msgs::VelodyneScan::ConstPtr &msg) {
  pcl_types::LidarScanStamped result;
  result.stamp = msg->header.stamp.toSec();

  for (const auto &packet : msg->packets) {
    const double packetTimeOffset = packet.stamp.toSec() - result.stamp;
    constexpr float kTWO_PI = std::numbers::pi_v<float> * 2.0f;
    constexpr float kCENTI_TO_UNIT = 0.01f;
    constexpr float kMILLI_TO_UNIT = 0.001f;
    constexpr std::size_t kFLAG_SIZE = 2;
    constexpr std::size_t kAZIMUTH_SIZE = 2;
    constexpr std::size_t kRANGE_SIZE = 2;
    constexpr std::size_t kINTENSITY_SIZE = 1;
    constexpr size_t kNUM_BLOCKS = 12;
    constexpr size_t kSEQUENCES_PER_BLOCK = 1;
    constexpr size_t kBLOCK_SIZE =
        kFLAG_SIZE + kAZIMUTH_SIZE +
        kSEQUENCES_PER_BLOCK * kNUM_CHANNELS * (kRANGE_SIZE + kINTENSITY_SIZE);
    constexpr float kCHANNEL_TIME = 2.304e-6f;
    constexpr float kRECHARGE_TIME = 18.432e-6f;
    constexpr size_t kNUM_FIRINGS = 16;
    constexpr float kSEQUENCE_TIME =
        kCHANNEL_TIME * kNUM_FIRINGS + kRECHARGE_TIME;
    constexpr float kBLOCK_TIME = kSEQUENCE_TIME * kSEQUENCES_PER_BLOCK;

    size_t index = 0;
    for (size_t block = 0; block < kNUM_BLOCKS; ++block) {
      index += kFLAG_SIZE;
      const float blockAzimuth =
          static_cast<float>(getBytes<kAZIMUTH_SIZE>(&packet.data[index])) *
              kCENTI_TO_UNIT * kDEG_TO_RAD +
          std::numbers::pi_v<float> / 2.0f;

      float azimuthRate = 0.0;
      if (m_azimuth >= 0.0) {
        azimuthRate = (blockAzimuth - m_azimuth);
        if (azimuthRate < 0) {
          azimuthRate += kTWO_PI;
        }
        azimuthRate /= kSEQUENCE_TIME;
      }

      index += kAZIMUTH_SIZE;
      for (size_t sequence = 0; sequence < kSEQUENCES_PER_BLOCK; ++sequence) {
        for (size_t channel = 0; channel < kNUM_CHANNELS; ++channel) {
          const float range =
              static_cast<float>(static_cast<uint16_t>(2) *
                                 getBytes<kRANGE_SIZE>(&packet.data[index])) *
              kMILLI_TO_UNIT;
          index += kRANGE_SIZE;
          const uint8_t intensity =
              getBytes<kINTENSITY_SIZE>(&packet.data[index]);
          index += kINTENSITY_SIZE;
          const float timeOffset =
              packetTimeOffset + kBLOCK_TIME * static_cast<float>(block) +
              kSEQUENCE_TIME * static_cast<float>(sequence) +
              kCHANNEL_TIME * static_cast<float>(channel);
          float preciseAzimuth =
              blockAzimuth +
              azimuthRate * (kCHANNEL_TIME * static_cast<float>(channel) +
                             kSEQUENCE_TIME * sequence);

          const float xyRange = range * m_channelToCosVerticalAngle[channel];
          result.cloud.push_back(
              {.x = xyRange * std::sin(preciseAzimuth),
               .y = xyRange * std::cos(preciseAzimuth),
               .z = range * m_channelToSinVerticalAngle[channel],
               .intensity = intensity,
               .channel = static_cast<uint8_t>(channel),
               .timeOffset = timeOffset});
          if (!result.cloud.back().getVector3fMap().allFinite()) {
            result.cloud.pop_back();
          }
        }
      }
    }
  }
  return result;
}

} // namespace velodyne_decoder
