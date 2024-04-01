#include "pcl_types/pcl_types_ros1.hpp"
#include "point_lio_ros1/point_lio_ros1.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <gtsam/navigation/NavState.h>
#include 

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
  ros::Publisher imu_pub = nh.advertise<StateInfo>("imu_data",10);

  rosbag::Bag bag(argv[1], rosbag::bagmode::Read);

  std::vector<std::string> topics{"/cmu_rc2/imu/data",
                                  "/cmu_rc2/velodyne_packets/point_cloud"};

  point_lio::PointLIO pointLIO;

  for (const auto &msg : rosbag::View(bag)) {
    const auto dataType = msg.getDataType();

    if(dataType == "sensor_msgs/Imu")
    {
      const auto imuMsg = msg.instantiate<sensor_msgs::Imu>();
      if (imuMsg != nullptr) {
        point_lio::Imu imu;
        if (point_lio::ros1::fromMsg(*imuMsg, imu)) {
          const auto odometry = pointLIO.registerImu(imu);
          odometry.print();

          // Converting odometry from NavState type to StateInfo custom msg type and publish it
          imu_pub.publish(NavstateToStateinfo(odometry));

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
        }
      }
    }
  }

  ros::spin();

  return 0;
}
