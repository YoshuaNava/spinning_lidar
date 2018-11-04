
#include "spinning_lidar_utils/laser_range_filter.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_range_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("LaserRangeFilter: Initializing node...");
  spinning_lidar_utils::LaserRangeFilter laser_range_filter(nh, pnh);

  ros::spin();

  return EXIT_SUCCESS;
}
