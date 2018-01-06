
#include "utils/pose6DOF.h"
#include "icpslam/octree_mapper.h"


OctreeMapper::OctreeMapper(ros::NodeHandle nh) :
	nh_(nh),
	map_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
	map_octree_(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION))
{
	init();
}

void OctreeMapper::init()
{
	initMap();
	loadParameters();
	advertisePublishers();
	registerSubscribers();

	ROS_INFO("Octree mapper started");
}

void OctreeMapper::loadParameters()
{
	nh_.param("verbosity_level", verbosity_level_, 2);

	// TF frames
	nh_.param("map_frame", map_frame_, std::string("map"));
	nh_.param("odom_frame", odom_frame_, std::string("odom"));
	nh_.param("robot_frame", robot_frame_, std::string("base_link"));
	nh_.param("laser_frame", laser_frame_, std::string("laser"));

	// Input robot odometry and point cloud topics
	nh_.param("increment_cloud_topic", increment_cloud_topic_, std::string("spinning_lidar/increment_cloud"));
	nh_.param("map_cloud_topic", map_cloud_topic_, std::string("icpslam/map_cloud"));
}

void OctreeMapper::advertisePublishers()
{
	map_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_cloud_topic_, 1);
}

void OctreeMapper::registerSubscribers()
{
	increment_cloud_sub_ = nh_.subscribe(increment_cloud_topic_, 10, &OctreeMapper::incrementCloudCallback, this);
}

void OctreeMapper::initMap()
{
	map_octree_->setInputCloud(map_cloud_);
}

void OctreeMapper::resetMap()
{
	map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
	map_octree_.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION));
	map_octree_->setInputCloud(map_cloud_);
}

/* This function is inspired on https://github.com/erik-nelson/point_cloud_mapper */
void OctreeMapper::addPointsToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	for(size_t i=0; i<input_cloud->points.size() ;i++)
	{
		pcl::PointXYZ point = input_cloud->points[i];
		if(!map_octree_->isVoxelOccupiedAtPoint(point))
		{
			map_octree_->addPointToCloud(point, map_cloud_);
		}
	}
}

void OctreeMapper::transformCloudToFixedFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, std::string target_frame, std::string source_frame, ros::Time target_time)
{
    if(tf_listener_.canTransform(target_frame, source_frame, target_time))
        try
        {
            tf::StampedTransform transform;
            tf_listener_.lookupTransform(target_frame, source_frame, target_time, transform);
            pcl_ros::transformPointCloud(*cloud_in, *cloud_out, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    else
        ROS_INFO("Transform from %s to %s not available!", source_frame.c_str(), target_frame.c_str());
}

void OctreeMapper::addCloudToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::string source_frame)
{
	// if(source_frame != map_frame_)
	// {
	// 	pcl::PointCloud<pcl::PointXYZ>::Ptr new_points_map(new pcl::PointCloud<pcl::PointXYZ>());
	// 	transformCloudToFixedFrame(cloud_in, new_points_map, map_frame_, robot_frame_, ros::Time(0));
	// 	addPointsToMap(new_points_map);
	// }
	// else
	// {
	// 	addPointsToMap(cloud_in);
	// }
	addPointsToMap(cloud_in);
}

void OctreeMapper::incrementCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	ROS_INFO("Map increment cloud callback!");
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>()), new_points(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::fromROSMsg(*cloud_msg, *input_cloud);

		// This should be done before
		pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
		voxel_filter.setInputCloud(input_cloud);
		voxel_filter.setLeafSize(0.05, 0.05, 0.05);
		voxel_filter.filter(*new_points);

	addCloudToMap(new_points, cloud_msg->header.frame_id);

	publishPointCloud(map_cloud_, map_frame_, ros::Time().now(), &map_cloud_pub_);
}
