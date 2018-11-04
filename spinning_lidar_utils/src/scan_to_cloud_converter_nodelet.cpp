
#include "scan_to_cloud_converter_nodelet.hpp"

// ROS pluginlib
#include <pluginlib/class_list_macros.h>

namespace spinning_lidar_utils {

ScanToCloudConverter::ScanToCloudConverter() : tf_listener_(tf_buffer_) {}

void ScanToCloudConverterNodelet::onInit() {
  this->ScanToCloudConverter::nh_ = getNodeHandle();
  this->ScanToCloudConverter::pnh_ = getPrivateNodeHandle();
  NODELET_INFO("ScanToCloudConverter: Initializing nodelet...");
  getParameters();
  initRosTransport();
}

}  // namespace spinning_lidar_utils

PLUGINLIB_EXPORT_CLASS(spinning_lidar_utils::ScanToCloudConverterNodelet, nodelet::Nodelet)