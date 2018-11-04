
#include "spinning_lidar_utils/interrupt_laser_assembler.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "interrupt_laser_assembler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("InterruptLaserAssembler: Initializing node...");
  spinning_lidar_utils::InterruptLaserAssembler laser_assembler(nh, pnh);

  ros::spin();

  return EXIT_SUCCESS;
}
