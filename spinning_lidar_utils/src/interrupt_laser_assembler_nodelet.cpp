
#include "interrupt_laser_assembler_nodelet.hpp"

// ROS pluginlib
#include <pluginlib/class_list_macros.h>

namespace spinning_lidar_utils {

InterruptLaserAssembler::InterruptLaserAssembler() {}

void InterruptLaserAssemblerNodelet::onInit() {
  this->InterruptLaserAssembler::nh_ = getNodeHandle();
  this->InterruptLaserAssembler::pnh_ = getPrivateNodeHandle();
  NODELET_INFO("InterruptLaserAssembler: Initializing nodelet...");
  getParameters();
  initRosTransport();
}

}  // namespace spinning_lidar_utils

PLUGINLIB_EXPORT_CLASS(spinning_lidar_utils::InterruptLaserAssemblerNodelet, nodelet::Nodelet)