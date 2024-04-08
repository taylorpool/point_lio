#include "pcl_types/pcl_types_ros1.hpp"
#include "point_lio_ros1/point_lio_ros1.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "point_lio/point_lio.hpp"

#include <span>
#include <thread>

int main(int argc, char *argv[]) noexcept {
  const std::span args(argv, argc);

  if (args.size() < 2) {
    std::cout << "./run_bag <bag-name>\n";
    return 1;
  }

  // Initializing ROS stuff
  ros::init(argc, argv, "imu_dead_reckoning");
  ros::NodeHandle nh;

  // Setting up publishers
  ros::Publisher imu_pub = nh.advertise<nav_msgs::Odometry>("imu_data",10);

  rosbag::Bag bag(argv[1], rosbag::bagmode::Read);

  std::vector<std::string> topics{"/cmu_rc2/imu/data",
                                  "/cmu_rc2/velodyne_packets/point_cloud"};

  // Creating PointLIO class object
  point_lio::PointLIO pointLIO;

  // Creating EKF class object
  point_lio::EKF ekf;

  for (const auto &msg : rosbag::View(bag)) {
    const auto dataType = msg.getDataType();

    ekf.predict(ekf.state, pointLIO.delt, ekf.P);

    if(dataType == "sensor_msgs/Imu")
    {
      const auto imuMsg = msg.instantiate<sensor_msgs::Imu>();
      if (imuMsg != nullptr) {
        point_lio::Imu imu;
        if (point_lio::ros1::fromMsg(*imuMsg, imu)) {
          const auto odometry = pointLIO.registerImu(pointLIO.imu_state, imu);
          odometry.print();
          // Converting odometry from NavState type to StateInfo custom msg type and publish it
          imu_pub.publish(pointLIO.NavstateToOdometry(odometry));

          ekf.updateIMU(ekf.state, pointLIO.imu_state, ekf.P);

          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
      }
    }
    else if(dataType == "sensor_msgs/PointCloud")
    {
      const auto scanMsg = msg.instantiate<sensor_msgs::PointCloud2>();
      if (scanMsg != nullptr) {
        pcl_types::LidarScanStamped scan;
        if (pcl_types::ros1::fromMsg(*scanMsg, scan)) {
          const auto map = pointLIO.registerScan(scan);

          ekf.updateLIDAR(ekf.state, ekf.plane, ekf.P, ekf.CorrectedIMUState);
        }
      }
    }
  }

  ros::spin();

  return 0;
}
