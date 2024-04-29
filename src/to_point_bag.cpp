#include "velodyne_decoder/velodyne_decoder_ros.hpp"

#include "velodyne_decoder/vlp16.hpp"

#include <pcl_types/pcl_types_ros1.hpp>
#include <utils_ros1/utils_ros1.hpp>

#include <iostream>
#include <span>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char *argv[]) {
  const std::span args(argv, argc);
  if (args.size() < 3) {
    std::cout << "./to_point_bag <bag-name> <output-name>\n";
    return 1;
  }

  rosbag::Bag inputBag(args[1], rosbag::bagmode::Read);
  rosbag::Bag outputBag(args[2], rosbag::bagmode::Write);

  const std::string pointTopic = "/cmu_rc2/points";
  const std::string imuTopic = "/cmu_rc2/imu/data";

  std::deque<boost::shared_ptr<sensor_msgs::Imu>> imuBuffer;
  std::deque<ros::Time> imuTimes;

  for (const auto &msg : rosbag::View(inputBag)) {
    const auto topic = msg.getTopic();
    const auto dataType = msg.getDataType();

    if (dataType == "sensor_msgs/Imu") {
      std::cout << "imu\n";
      const auto imuMsg = msg.instantiate<sensor_msgs::Imu>();
      if (imuMsg != nullptr) {
        imuBuffer.push_back(imuMsg);
        imuTimes.push_back(msg.getTime());
      }
    } else if (dataType == "sensor_msgs/PointCloud2") {
      const auto scanMsg = msg.instantiate<sensor_msgs::PointCloud2>();
      std::cout << "lidar\n";
      if (scanMsg != nullptr) {
        pcl_types::LidarScanStamped scan;
        if (pcl_types::ros1::fromMsg(*scanMsg, scan)) {
          for (const auto &point : scan.cloud) {
            geometry_msgs::Vector3Stamped outMsg;
            utils::ros1::toMsg(point.timeOffset + scanMsg->header.stamp.toSec(),
                               outMsg.header.stamp);
            utils::ros1::toMsg(point.getVector3fMap().cast<double>(),
                               outMsg.vector);
            while (!imuBuffer.empty() &&
                   imuBuffer.front()->header.stamp < outMsg.header.stamp) {
              outputBag.write(imuTopic, imuBuffer.front()->header.stamp,
                              imuBuffer.front());
              imuTimes.pop_front();
              imuBuffer.pop_front();
            }

            outputBag.write(pointTopic, outMsg.header.stamp, outMsg);
          }
        }
      }
    }
  }
  return 0;
}
