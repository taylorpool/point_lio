#include "pcl_types/pcl_types_ros1.hpp"
#include "point_lio_ros1/point_lio_ros1.hpp"

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "point_lio");
  ros::NodeHandle publicNode;
  ros::NodeHandle privateNode;

  point_lio::PointLIO pointLIO;

  auto odometryPublisher =
      publicNode.advertise<nav_msgs::Odometry>("/odometry", 2);

  auto imuSubscriber = publicNode.subscribe<sensor_msgs::Imu>(
      "/cmu_rc2/imu/data", 10, [&](const sensor_msgs::Imu::ConstPtr &imuMsg) {
        point_lio::Imu imu;
        if (point_lio::ros1::fromMsg(*imuMsg, imu)) {
          pointLIO.registerImu(imu);
        }
      });

  auto scanSubscriber = publicNode.subscribe<sensor_msgs::PointCloud2>(
      "/cmu_rc2/velodyne_packets/point_cloud", 10,
      [&](const sensor_msgs::PointCloud2::ConstPtr &scanMsg) {
        pcl_types::LidarScanStamped scan;
        if (pcl_types::ros1::fromMsg(*scanMsg, scan)) {
          pointLIO.registerScan(scan);
        }
      });

  ros::spin();

  return 0;
}
