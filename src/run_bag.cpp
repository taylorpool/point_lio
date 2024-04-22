#include "pcl_types/pcl_types_ros1.hpp"
#include "point_lio_ros1/point_lio_ros1.hpp"

#include "point_lio/point_lio.hpp"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <span>

int main(int argc, char *argv[]) noexcept {
  const std::span args(argv, argc);

  if (args.size() < 3) {
    std::cout << "./run_bag <input-bag-name> <output-bag-name>\n";
    return 1;
  }

  const std::string odometryTopic = "/odometry";

  rosbag::Bag inputbag(argv[1], rosbag::bagmode::Read);
  rosbag::Bag outputBag(argv[2], rosbag::bagmode::Write);

  point_lio::PointLIO pointLIO;

  for (const auto &msg : rosbag::View(inputbag)) {
    const auto topic = msg.getTopic();
    const auto dataType = msg.getDataType();

    if (dataType == "sensor_msgs/Imu") {
      const auto imuMsg = msg.instantiate<sensor_msgs::Imu>();
      if (imuMsg != nullptr) {
        point_lio::Imu imu;
        if (point_lio::ros1::fromMsg(*imuMsg, imu)) {
          pointLIO.registerImu(imu);

          {
            nav_msgs::Odometry odomMsg;
            odomMsg.header.frame_id = "world";
            odomMsg.header.stamp = imuMsg->header.stamp;
            odomMsg.pose.pose.position.x = pointLIO.world_position.x();
            odomMsg.pose.pose.position.y = pointLIO.world_position.y();
            odomMsg.pose.pose.position.z = pointLIO.world_position.z();
            const auto tmp = pointLIO.world_R_body.toQuaternion();
            odomMsg.pose.pose.orientation.w = tmp.w();
            odomMsg.pose.pose.orientation.x = tmp.x();
            odomMsg.pose.pose.orientation.y = tmp.y();
            odomMsg.pose.pose.orientation.z = tmp.z();

            outputBag.write(odometryTopic, msg.getTime(), odomMsg);
          }
        }
      }
    } else if (dataType == "sensor_msgs/PointCloud") {
      const auto scanMsg = msg.instantiate<sensor_msgs::PointCloud2>();
      if (scanMsg != nullptr) {
        pcl_types::LidarScanStamped scan;
        if (pcl_types::ros1::fromMsg(*scanMsg, scan)) {
          pointLIO.registerScan(scan);

          {
            nav_msgs::Odometry odomMsg;
            odomMsg.header.frame_id = "world";
            odomMsg.header.stamp = imuMsg->header.stamp;
            odomMsg.pose.pose.position.x = pointLIO.world_position.x();
            odomMsg.pose.pose.position.y = pointLIO.world_position.y();
            odomMsg.pose.pose.position.z = pointLIO.world_position.z();
            const auto tmp = pointLIO.world_R_body.toQuaternion();
            odomMsg.pose.pose.orientation.w = tmp.w();
            odomMsg.pose.pose.orientation.x = tmp.x();
            odomMsg.pose.pose.orientation.y = tmp.y();
            odomMsg.pose.pose.orientation.z = tmp.z();

            outputBag.write(odometryTopic, msg.getTime(), odomMsg);
          }
        }
      }
    }
  }

  return 0;
}
