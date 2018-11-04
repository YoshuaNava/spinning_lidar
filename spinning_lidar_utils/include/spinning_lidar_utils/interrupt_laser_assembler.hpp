#pragma once

// ros
#include <ros/ros.h>

// ros msgs
#include <std_msgs/Empty.h>

// laser assembler
#include <laser_assembler/AssembleScans2.h>

namespace spinning_lidar_utils {

class InterruptLaserAssembler {
 public:
  InterruptLaserAssembler();

  InterruptLaserAssembler(ros::NodeHandle nh, ros::NodeHandle pnh);

 protected:
  void irInterruptCallback(const std_msgs::Empty::ConstPtr& msg);

  void getParameters();

  void initRosTransport();

  int min_num_points_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string ir_interrupt_topic_;
  std::string assembled_cloud_topic_;
  std::string assemble_service_;

  laser_assembler::AssembleScans2 assemble_srv_;
  ros::ServiceClient assemble_client_;

  ros::Subscriber ir_interrupt_sub_;
  ros::Publisher point_cloud_pub_;
};

}  // namespace spinning_lidar_utils