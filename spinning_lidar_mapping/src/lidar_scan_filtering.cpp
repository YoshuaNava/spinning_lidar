

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>


float inf = std::numeric_limits<float>::infinity();
ros::Publisher filtered_scan_pub;
int num_points_thesh = 5000;
std::string laser_scan_topic, filtered_scan_topic, laser_link;
double min_dist_to_sensor;


class DynamicLaserToPointCloud
{
public:
	ros::NodeHandle nh_;
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;

	DynamicLaserToPointCloud(ros::NodeHandle nh, std::string laser_scan_topic, std::string laser_link, std::string filtered_scan_topic, double min_dist_to_sensor) :
		nh_(nh),
		laser_sub_(nh_, laser_scan_topic, 10),
		tf_laser_filter_(laser_sub_, tf_listener_, laser_link, 10)
	{
		laser_scan_topic_ = laser_scan_topic;
		laser_link_ = laser_link;
		filtered_scan_topic_ = filtered_scan_topic;
		tf_laser_filter_.setTolerance(ros::Duration(tf_filter_tol_));
		tf_laser_filter_.registerCallback( boost::bind(&DynamicLaserToPointCloud::scanCallback, this, _1) );
		filtered_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(filtered_scan_topic, 1);
		min_dist_to_sensor_ = min_dist_to_sensor;
	}

private:
	
	ros::Publisher filtered_scan_pub_;
	std::string laser_scan_topic_, laser_link_, filtered_scan_topic_;
	tf::TransformListener tf_listener_;
	tf::MessageFilter<sensor_msgs::LaserScan> tf_laser_filter_;
	laser_geometry::LaserProjection laser_projector_;
	double tf_filter_tol_ = 0.01;
	double min_dist_to_sensor_;
	
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{
		sensor_msgs::LaserScan filtered_scan; //create new LaserScan msg for filtered points

		// Reference: https://github.com/RobustFieldAutonomyLab/spin_hokuyo/blob/master/src/hokuyo_robot_filter.cpp
		int num_range_meas = scan->ranges.size();
		int num_int_meas = scan->intensities.size();
		filtered_scan.header = scan->header;
		filtered_scan.angle_min = scan->angle_min;
		filtered_scan.angle_max = scan->angle_max;
		filtered_scan.angle_increment = scan->angle_increment;
		filtered_scan.time_increment = scan->time_increment;
		filtered_scan.scan_time = scan->scan_time;
		filtered_scan.range_min = 0;
		filtered_scan.range_max = inf;
		filtered_scan.intensities[num_int_meas];
		filtered_scan.ranges.resize(num_range_meas);

		for(int n=0; n<num_range_meas ;n++)
		{
			if (scan->ranges[n] > min_dist_to_sensor_)
			{ 
				filtered_scan.ranges[n] = scan->ranges[n];
			}
			else
			{
				// Set range to inf to "remove" point if it is so close that it could be the laser platform
				filtered_scan.ranges[n] = inf;
			}
		}

		filtered_scan_pub_.publish(filtered_scan);
	}
};




int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_scan_filtering");
	ros::NodeHandle nh;

	nh.param("min_dist_to_sensor", min_dist_to_sensor, 0.5);
	nh.param("laser_link", laser_link, std::string("laser"));
	nh.param("laser_scan_topic", laser_scan_topic, std::string("spinning_lidar/scan"));
	nh.param("filtered_scan_topic", filtered_scan_topic, std::string("spinning_lidar/filtered_scan"));

	ROS_INFO("Dynamic (using TF) filtering of laser scans");
	DynamicLaserToPointCloud laser_pcl_converter(nh, laser_scan_topic, laser_link, filtered_scan_topic, min_dist_to_sensor);
		
	ros::spin();

	return EXIT_SUCCESS;
}
