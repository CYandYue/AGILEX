#include <ros/ros.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <vector>

// Only label each device's raw frame. Do not apply spatial transforms, change
// timestamps, drop point fields, or convert IMU units here.
int main(int argc, char** argv) {
  ros::init(argc, argv, "lio_sensor_relay");
  ros::NodeHandle nh, pnh("~");
  std::vector<std::string> topics;
  if (!pnh.getParam("input_topics", topics) || topics.size() != 4) {
    ROS_FATAL("input_topics must contain lidar A, lidar B, imu A, imu B");
    return 1;
  }
  std::vector<ros::Publisher> pubs;
  std::vector<ros::Subscriber> subs;
  for (int i = 0; i < 4; ++i) {
    const std::string label = i % 2 == 0 ? "a" : "b";
    const std::string kind = i < 2 ? "lidar" : "imu";
    const std::string frame = "mid360_" + label + "_" + kind;
    const std::string output = "/sensors/mid360_" + label + "/" + kind;
    if (i < 2) {
      auto pub = nh.advertise<livox_ros_driver2::CustomMsg>(output, 20);
      pubs.push_back(pub);
      subs.push_back(nh.subscribe<livox_ros_driver2::CustomMsg>(topics[i], 20,
        [pub, frame](const livox_ros_driver2::CustomMsg::ConstPtr& input) {
          auto output = *input;
          output.header.frame_id = frame;
          pub.publish(output);
        }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
    } else {
      auto pub = nh.advertise<sensor_msgs::Imu>(output, 200);
      pubs.push_back(pub);
      subs.push_back(nh.subscribe<sensor_msgs::Imu>(topics[i], 200,
        [pub, frame](const sensor_msgs::Imu::ConstPtr& input) {
          auto output = *input;
          output.header.frame_id = frame;
          pub.publish(output);
        }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
    }
  }
  ros::spin();
  return 0;
}
