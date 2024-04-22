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

  std::deque<point_lio::Imu> imuBuffer;

  auto imuSubscriber = publicNode.subscribe<sensor_msgs::Imu>(
      "/cmu_rc2/imu/data", 10, [&](const sensor_msgs::Imu::ConstPtr &imuMsg) {
        point_lio::Imu imu;
        if (point_lio::ros1::fromMsg(*imuMsg, imu)) {
          imuBuffer.push_back(imu);
        }
      });

  auto scanSubscriber = publicNode.subscribe<sensor_msgs::PointCloud2>(
      "/cmu_rc2/velodyne_packets/point_cloud", 10,
      [&](const sensor_msgs::PointCloud2::ConstPtr &scanMsg) {
        pcl_types::LidarScanStamped scan;
        if (pcl_types::ros1::fromMsg(*scanMsg, scan)) {
          for (const auto &point : scan.cloud) {
            pcl_types::PointXYZICT newPoint = point;
            newPoint.timeOffset += scan.stamp;
            while (!imuBuffer.empty() &&
                   imuBuffer.front().stamp < newPoint.timeOffset) {
              pointLIO.registerImu(imuBuffer.front());
              imuBuffer.pop_front();
            }
            pointLIO.registerPoint(newPoint);
          }
        }
      });

  ros::spin();

  return 0;
}
