#include "point_lio_ros1/point_lio_ros1.hpp"

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>

int main(int argc, char *argv[]) {

  std::cout << "hello from main\n";

  ros::init(argc, argv, "point_lio");
  ros::NodeHandle publicNode;
  ros::NodeHandle privateNode;

  point_lio::PointLIO pointLIO;

  auto odometryPublisher =
      publicNode.advertise<nav_msgs::Odometry>("/odometry", 2);

  auto imuSubscriber = publicNode.subscribe<sensor_msgs::Imu>(
      "/cmu_rc2/imu/data", 10, [&](const sensor_msgs::Imu::ConstPtr &imuMsg) {

      });

  auto scanSubscriber = publicNode.subscribe<sensor_msgs::PointCloud2>(
      "/cmu_rc2/velodyne_packets/point_cloud", 10,
      [&](const sensor_msgs::PointCloud2::ConstPtr &scanMsg) {

      });

  ros::spin();

  return 0;
}
