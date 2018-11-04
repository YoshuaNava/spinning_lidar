#pragma once

// ros
#include <ros/ros.h>

// sensor msgs
#include <sensor_msgs/LaserScan.h>

namespace spinning_lidar_utils {

class LaserRangeFilter {
 public:
  LaserRangeFilter();

  LaserRangeFilter(ros::NodeHandle nh, ros::NodeHandle pnh);

 protected:
  void getParameters();

  void initRosTransport();

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  double min_dist_to_sensor_;
  double max_dist_to_sensor_;
  float inf = std::numeric_limits<float>::infinity();

  std::string input_scan_topic_;
  std::string output_scan_topic_;

  ros::Subscriber laser_sub_;
  ros::Publisher output_scan_pub_;
};

}  // namespace spinning_lidar_utils