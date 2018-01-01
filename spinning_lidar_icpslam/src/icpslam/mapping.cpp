
#include "icpslam/mapping.h"


void incrementCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>()), new_points(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::fromROSMsg(*cloud_msg, *input_cloud);

		// This should be done before
		pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
		voxel_filter.setInputCloud(input_cloud);
		voxel_filter.setLeafSize(0.05, 0.05, 0.05);
		voxel_filter.filter(*new_points);

	pcl::PointCloud<pcl::PointXYZ>::Ptr new_points_map(new pcl::PointCloud<pcl::PointXYZ>());
	transformCloudToFixedFrame(new_points, new_points_map, map_frame_, robot_frame_, ros::Time(0), tf_listener_ptr_);
	addPointsToMap(new_points_map);

	publishPointCloud(map_cloud_, map_frame_, ros::Time().now(), &map_cloud_pub_);
}

void loadParameters(ros::NodeHandle nh)
{
	nh.param("verbosity_level", verbosity_level_, 2);

	// TF frames
	nh.param("map_frame", map_frame_, std::string("odom"));
	nh.param("odom_frame", odom_frame_, std::string("odom"));
	nh.param("robot_frame", robot_frame_, std::string("base_link"));
	nh.param("laser_frame", laser_frame_, std::string("laser"));

	// Input robot odometry and point cloud topics
	nh.param("increment_cloud_topic", increment_cloud_topic_, std::string("spinning_lidar/assembled_cloud"));
	nh.param("map_cloud_topic", map_cloud_topic_, std::string("icpslam/map_cloud"));
}

void advertisePublishers(ros::NodeHandle nh)
{
	map_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(map_cloud_topic_, 1);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "icpslam_mapping");
	ros::NodeHandle nh;

	loadParameters(nh);
	advertisePublishers(nh);
	
	tf::TransformListener tf_listener;
	tf_listener_ptr_ = &tf_listener;
	tf::TransformBroadcaster tf_broadcaster;
	tf_broadcaster_ptr_ = &tf_broadcaster;

	initMap(map_frame_, odom_frame_, tf_broadcaster_ptr_);

	ros::Subscriber assembled_cloud_sub = nh.subscribe(increment_cloud_topic_, 1, incrementCloudCallback);

	ROS_INFO("ICPSLAM mapping started");	

	ros::spin();

	return EXIT_SUCCESS;
}