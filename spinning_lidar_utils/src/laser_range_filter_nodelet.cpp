
#include "laser_range_filter_nodelet.hpp"

// ROS pluginlib
#include <pluginlib/class_list_macros.h>

namespace spinning_lidar_utils {

LaserRangeFilter::LaserRangeFilter() {}

void LaserRangeFilterNodelet::onInit() {
  this->LaserRangeFilter::nh_ = getNodeHandle();
  this->LaserRangeFilter::pnh_ = getPrivateNodeHandle();
  NODELET_INFO("LaserRangeFilter: Initializing nodelet...");
  getParameters();
  initRosTransport();
}

}  // namespace spinning_lidar_utils

PLUGINLIB_EXPORT_CLASS(spinning_lidar_utils::LaserRangeFilterNodelet, nodelet::Nodelet)