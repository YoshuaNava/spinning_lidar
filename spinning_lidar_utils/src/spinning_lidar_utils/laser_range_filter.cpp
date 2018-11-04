
#include "spinning_lidar_utils/laser_range_filter.hpp"

namespace spinning_lidar_utils {

LaserRangeFilter::LaserRangeFilter(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh) {
  getParameters();
  initRosTransport();
}

void LaserRangeFilter::getParameters() {
  pnh_.param("min_dist_to_sensor", min_dist_to_sensor_, 0.5);
  pnh_.param("max_dist_to_sensor", max_dist_to_sensor_, 30.0);
  pnh_.param("input_scan_topic", input_scan_topic_, std::string("scan"));
  pnh_.param("output_scan_topic", output_scan_topic_, std::string("filtered_scan"));
}

void LaserRangeFilter::initRosTransport() {
  laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(
      input_scan_topic_, 1, &LaserRangeFilter::scanCallback, this, ros::TransportHints().tcpNoDelay());

  output_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(output_scan_topic_, 1);
}

void LaserRangeFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  sensor_msgs::LaserScan filtered_scan;  // create new LaserScan msg for filtered points

  // LIDAR scan filtering
  // Reference: https://github.com/RobustFieldAutonomyLab/spin_hokuyo/blob/master/src/hokuyo_robot_filter.cpp
  size_t num_range_meas = scan->ranges.size();
  size_t num_int_meas = scan->intensities.size();
  filtered_scan.header = scan->header;
  filtered_scan.angle_min = scan->angle_min;
  filtered_scan.angle_max = scan->angle_max;
  filtered_scan.angle_increment = scan->angle_increment;
  filtered_scan.time_increment = scan->time_increment;
  filtered_scan.scan_time = scan->scan_time;
  filtered_scan.range_min = min_dist_to_sensor_;
  filtered_scan.range_max = max_dist_to_sensor_;
  filtered_scan.intensities.resize(num_int_meas);
  filtered_scan.ranges.resize(num_range_meas);

  for (size_t n = 0; n < num_range_meas; n++) {
    if ((scan->ranges[n] > min_dist_to_sensor_) && (scan->ranges[n] < max_dist_to_sensor_)) {
      filtered_scan.ranges[n] = scan->ranges[n];
      filtered_scan.intensities[n] = scan->intensities[n];
    } else {
      // Set range to inf to "remove" point
      filtered_scan.ranges[n] = inf;
      filtered_scan.intensities[n] = 0;
    }
  }
  output_scan_pub_.publish(filtered_scan);
}

}  // namespace spinning_lidar_utils