#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"


class LaserToPointCloud
{
public:
	ros::NodeHandle nh_;
	ros::Publisher point_cloud_pub_;
	laser_geometry::LaserProjection laser_projector_;
	std::string laser_scan_topic_, laser_link_, point_cloud_topic_;

	LaserToPointCloud(ros::NodeHandle nh, std::string laser_scan_topic, std::string laser_link, std::string point_cloud_topic) :
		nh_(nh)
	{ 
		laser_scan_topic_ = laser_scan_topic;
		laser_link_ = laser_link;
		point_cloud_topic_ = point_cloud_topic;
	}
};


class DynamicLaserToPointCloud : public LaserToPointCloud
{
public:
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;

	DynamicLaserToPointCloud(ros::NodeHandle nh, std::string laser_scan_topic, std::string laser_link, std::string point_cloud_topic) :
		LaserToPointCloud(nh, laser_scan_topic, laser_link, point_cloud_topic),
		laser_sub_(nh_, laser_scan_topic, 10),
		tf_laser_filter_(laser_sub_, tf_listener_, laser_link, 10)
	{
		tf_laser_filter_.setTolerance(ros::Duration(tf_filter_tol_));
		tf_laser_filter_.registerCallback( boost::bind(&DynamicLaserToPointCloud::scanCallback, this, _1) );
		point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>(point_cloud_topic, 1);
	}

private:
	tf::TransformListener tf_listener_;
	tf::MessageFilter<sensor_msgs::LaserScan> tf_laser_filter_;
	double tf_filter_tol_ = 0.01;

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		// ROS_INFO("Received laser scan!");
		sensor_msgs::PointCloud cloud;
		try
		{
			laser_projector_.transformLaserScanToPointCloud(laser_link_, *scan_in, cloud, tf_listener_);
		}
		catch (tf::TransformException& e)
		{
			std::cout << e.what();
			return;
		}

		point_cloud_pub_.publish(cloud);
	}
};


class StaticLaserToPointCloud : public LaserToPointCloud
{
public:
	ros::Subscriber laser_sub_;
	StaticLaserToPointCloud(ros::NodeHandle nh, std::string laser_scan_topic, std::string laser_link, std::string point_cloud_topic) :
		LaserToPointCloud(nh, laser_scan_topic, laser_link, point_cloud_topic)
	{
		point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>(point_cloud_topic_, 1);
		laser_sub_ = nh.subscribe(laser_scan_topic_, 10, &StaticLaserToPointCloud::scanCallback, this);
	}

private:
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		// ROS_INFO("Received laser scan!");
		sensor_msgs::PointCloud cloud;
		laser_projector_.projectLaser(*scan_in, cloud);
		point_cloud_pub_.publish(cloud);
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_pcl_gen_node");
	ros::NodeHandle nh;

	// Parameters for the PR2 tilting laser dataset
	// std::string laser_scan_topic = "base_scan";
	// std::string laser_link = "base_laser_link";

	// Parameters for the Clearpath Ridgeback robot
	std::string laser_scan_topic = "front/scan";
	std::string laser_link = "front_laser";

	std::string point_cloud_topic = "my_point_cloud";
	bool use_tf = true;

	LaserToPointCloud* laser_pcl_converter;
	if(use_tf)
	{
		ROS_INFO("Dynamic (using TF) transformation of laser scans into point clouds");
		laser_pcl_converter = new DynamicLaserToPointCloud(nh, laser_scan_topic, laser_link, point_cloud_topic);
	}
	else
	{
		ROS_INFO("Static (not using TF) transformation of laser scans into point clouds");
		laser_pcl_converter = new StaticLaserToPointCloud(nh, laser_scan_topic, laser_link, point_cloud_topic);
	}
	
	ros::spin();

	return EXIT_SUCCESS;
}
