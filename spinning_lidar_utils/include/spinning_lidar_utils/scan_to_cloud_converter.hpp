#pragma once

// ros
#include <ros/ros.h>

// sensor msgs
#include <sensor_msgs/LaserScan.h>

// tf
#include <tf2_ros/transform_listener.h>

// laser geometry
#include <laser_geometry/laser_geometry.h>

namespace spinning_lidar_utils {

class ScanToCloudConverter {
 public:
  ScanToCloudConverter();

  ScanToCloudConverter(ros::NodeHandle nh, ros::NodeHandle pnh);

 protected:
  void getParameters();

  void initRosTransport();

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string target_frame_;
  std::string input_scan_topic_;
  std::string output_cloud_topic_;

  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;

  laser_geometry::LaserProjection laser_projector_;

  ros::Subscriber laser_sub_;
  ros::Publisher output_cloud_pub_;
};

}  // namespace spinning_lidar_utils