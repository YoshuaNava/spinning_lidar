
#include "icpslam/mapping.h"


void assembledCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>()), new_points(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::fromROSMsg(*cloud_msg, *input_cloud);

		// This should be done before
		pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
		voxel_filter.setInputCloud(input_cloud);
		voxel_filter.setLeafSize(0.05, 0.05, 0.05);
		voxel_filter.filter(*new_points);

	pcl::PointCloud<pcl::PointXYZ>::Ptr new_points_map(new pcl::PointCloud<pcl::PointXYZ>());
	transformCloudToFixedFrame(new_points, new_points_map, map_frame, robot_frame, ros::Time(0), tf_listener_ptr);
	addPointsToMap(new_points_map);
	publishPointCloud(map_cloud, map_frame, ros::Time().now(), &map_cloud_pub);
}

void loadParameters(ros::NodeHandle nh)
{
	nh.param("verbosity_level", verbosity_level, 2);

	// TF frames
	nh.param("map_frame", map_frame, std::string("map"));
	nh.param("odom_frame", odom_frame, std::string("odom"));
	nh.param("robot_frame", robot_frame, std::string("base_link"));
	nh.param("laser_frame", laser_frame, std::string("laser"));

	// Input robot odometry and point cloud topics
    nh.param("assembled_cloud_topic", assembled_cloud_topic, std::string("spinning_lidar/assembled_cloud"));
	nh.param("map_cloud_topic", map_cloud_topic, std::string("icpslam/map_cloud"));
}

void advertisePublishers(ros::NodeHandle nh)
{
	map_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(map_cloud_topic, 1);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "icpslam_mapping");
	ros::NodeHandle nh;

	loadParameters(nh);
	advertisePublishers(nh);
	
	tf::TransformListener tf_listener;
	tf_listener_ptr = &tf_listener;
	tf::TransformBroadcaster tf_broadcaster;
	tf_broadcaster_ptr = &tf_broadcaster;

	initMap(map_frame, odom_frame, tf_broadcaster_ptr);

	ros::Subscriber assembled_cloud_sub = nh.subscribe(assembled_cloud_topic, 1, assembledCloudCallback);

	ROS_INFO("ICP mapping started");	

	ros::spin();
	

	return EXIT_SUCCESS;
}