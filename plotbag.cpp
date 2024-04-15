#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <iostream>

int main() {
    rosbag::Bag bag;
    bag.open("imu_dead_reckoning_2024-04-03-22-45-33.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics = {"imu_data"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for (const rosbag::MessageInstance& msg : view) {
        sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
        if (imu_msg != nullptr) {
            // Process imu_msg data and plot as needed
            std::cout << "IMU data at time " << imu_msg->header.stamp << std::endl;
        }
    }

    bag.close();
    return 0;
}
