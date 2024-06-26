#include "velodyne_decoder/velodyne_decoder_ros.hpp"

#include "velodyne_decoder/vlp16.hpp"

#include <pcl_types/pcl_types_ros1.hpp>

#include <iostream>
#include <span>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char *argv[]) {
  const std::span args(argv, argc);
  if (args.size() < 2) {
    std::cout << "./vlp16_bag_converter <bag-name>\n";
    return 1;
  }

  velodyne_decoder::VLP16Decoder decoder;

  rosbag::Bag decodedBag("decoded.bag", rosbag::bagmode::Write);

  for (int index = 1; index < args.size(); ++index) {
    std::cout << index << "\r";
    rosbag::Bag bag(args[index], rosbag::bagmode::Read);

    for (const auto &msg : rosbag::View(bag)) {
      if (msg.getDataType() == "velodyne_msgs/VelodyneScan") {
        const auto &scan = msg.instantiate<velodyne_msgs::VelodyneScan>();
        const auto decodedScan = decoder.decode(scan);

        sensor_msgs::PointCloud2 cloud;
        pcl_types::ros1::toMsg(decodedScan, cloud);
        cloud.header = scan->header;
        decodedBag.write(msg.getTopic() + "/point_cloud", msg.getTime(), cloud);
      } else if (msg.getDataType() == "sensor_msgs/Imu") {
        decodedBag.write(msg.getTopic(), msg.getTime(),
                         msg.instantiate<sensor_msgs::Imu>());
      } else if (msg.getTopic() == "/ground_truth") {
        decodedBag.write(msg.getTopic(), msg.getTime(),
                         msg.instantiate<nav_msgs::Odometry>());
      }
    }
  }

  return 0;
}
