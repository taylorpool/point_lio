#include "point_lio_ros1/point_lio_ros1.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/PointCloud2.h>

int main(int argc, char *argv[]) {
  rosbag::Bag bag(argv[1], rosbag::bagmode::Read);

  std::vector<std::string> topics{"/cmu_rc2/imu/data",
                                  "/cmu_rc2/velodyne_packets/point_cloud"};

  point_lio::PointLIO pointLIO;

  for (const auto &msg : rosbag::View(bag)) {
    const auto imu = msg.instantiate<sensor_msgs::Imu>();
    if (imu != nullptr) {
    }

    const auto scan = msg.instantiate<sensor_msgs::PointCloud2>();
    if (scan != nullptr) {
    }
  }

  return 0;
}
