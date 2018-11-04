
#include "spinning_lidar_utils/scan_to_cloud_converter.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_scan_to_cloud");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("ScanToCloudConverter: Initializing node...");
  spinning_lidar_utils::ScanToCloudConverter converter(nh, pnh);

  ros::spin();

  return EXIT_SUCCESS;
}
