
#include "spinning_lidar_utils/scan_to_cloud_converter.hpp"

// sensor msgs
#include <sensor_msgs/PointCloud2.h>

// pcl tools
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace spinning_lidar_utils {

ScanToCloudConverter::ScanToCloudConverter(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_) {
  getParameters();
  initRosTransport();
}

void ScanToCloudConverter::getParameters() {
  pnh_.param("target_frame", target_frame_, std::string("laser"));
  pnh_.param("input_scan_topic", input_scan_topic_, std::string("filtered_scan"));
  pnh_.param("output_cloud_topic", output_cloud_topic_, std::string("filtered_cloud"));
}

void ScanToCloudConverter::initRosTransport() {
  laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(
      input_scan_topic_, 1, &ScanToCloudConverter::scanCallback, this, ros::TransportHints().tcpNoDelay());

  output_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1);
}

void ScanToCloudConverter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  if (!output_cloud_pub_.getNumSubscribers()) {
    return;
  }

  // Projection of laser scans into point clouds
  sensor_msgs::PointCloud2 cloud;
  try {
    laser_projector_.transformLaserScanToPointCloud(target_frame_, *scan, cloud, tf_buffer_);
    cloud.header = scan->header;
    cloud.header.frame_id = target_frame_;
    output_cloud_pub_.publish(cloud);
  } catch (tf::TransformException& e) {
    // std::cout << e.what();
    return;
  }
}

}  // namespace spinning_lidar_utils