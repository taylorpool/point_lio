#include "pcl_types/pcl_types_ros1.hpp"
#include "point_lio_ros1/point_lio_ros1.hpp"

#include <geometry_msgs/Vector3Stamped.h>

#include "point_lio/point_lio.hpp"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <span>
#include <thread>

int main(int argc, char *argv[]) noexcept {
  const std::span args(argv, argc);

  if (args.size() < 3) {
    std::cout << "./run_bag <input-bag-name> <output-bag-name>\n";
    return 1;
  }

  const std::string odometryTopic = "/odometry";

  rosbag::Bag inputbag(argv[1], rosbag::bagmode::Read);
  rosbag::Bag outputBag(argv[2], rosbag::bagmode::Write);

  point_lio::PointLIOParams params;
  params.imuInitializationQuota = 100;

  point_lio::PointLIO pointLIO(params);

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
            const auto world_R_body = pointLIO.world_R_body.toQuaternion();
            odomMsg.pose.pose.orientation.w = world_R_body.w();
            odomMsg.pose.pose.orientation.x = world_R_body.x();
            odomMsg.pose.pose.orientation.y = world_R_body.y();
            odomMsg.pose.pose.orientation.z = world_R_body.z();

            outputBag.write(odometryTopic, msg.getTime(), odomMsg);
          }
        }
      }
    } else if (dataType == "sensor_msgs/PointCloud2") {
      const auto scanMsg = msg.instantiate<sensor_msgs::PointCloud2>();
      if (scanMsg != nullptr) {
        pcl_types::LidarScanStamped scan;
        if (pcl_types::ros1::fromMsg(*scanMsg, scan)) {
          std::cout << "scan\n";
        }
      }
    }
  }

  return 0;
}
