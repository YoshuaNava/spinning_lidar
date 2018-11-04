
#include "spinning_lidar_utils/interrupt_laser_assembler.hpp"

// sensor msgs
#include <sensor_msgs/PointCloud2.h>

namespace spinning_lidar_utils {

InterruptLaserAssembler::InterruptLaserAssembler(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh) {
  getParameters();
  initRosTransport();
}

void InterruptLaserAssembler::getParameters() {
  pnh_.param("min_num_points", min_num_points_, 5000);
  pnh_.param("ir_interrupt_topic", ir_interrupt_topic_, std::string("ir_interrupt"));
  pnh_.param("assembled_cloud_topic", assembled_cloud_topic_, std::string("assembled_cloud"));
  pnh_.param("assemble_service", assemble_service_, std::string("assemble_scans2"));
}

void InterruptLaserAssembler::initRosTransport() {
  point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(assembled_cloud_topic_, 1);
  ir_interrupt_sub_ = nh_.subscribe<std_msgs::Empty>(
      ir_interrupt_topic_, 1, &InterruptLaserAssembler::irInterruptCallback, this, ros::TransportHints().tcpNoDelay());

  ros::service::waitForService(assemble_service_);
  assemble_client_ = nh_.serviceClient<laser_assembler::AssembleScans2>(assemble_service_);
}

void InterruptLaserAssembler::irInterruptCallback(const std_msgs::Empty::ConstPtr& msg) {
  if (!point_cloud_pub_.getNumSubscribers()) {
    return;
  }

  assemble_srv_.request.end = ros::Time::now();
  if (assemble_client_.call(assemble_srv_)) {
    size_t num_points = assemble_srv_.response.cloud.width;
    if (num_points > static_cast<unsigned int>(min_num_points_)) {
      point_cloud_pub_.publish(assemble_srv_.response.cloud);
      ROS_DEBUG("InterruptLaserAssembler: Got cloud with %lu points", num_points);
    }
  } else {
    ROS_WARN("InterruptLaserAssembler: Service call failed");
  }

  assemble_srv_.request.begin = assemble_srv_.request.end;
}

}  // namespace spinning_lidar_utils