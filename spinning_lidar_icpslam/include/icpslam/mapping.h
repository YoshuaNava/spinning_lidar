
#ifndef MAPPING_H
#define MAPPING_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>

#include "utils/geometric_utils.h"
#include "utils/messaging_utils.h"


// Constants for mapping
const float OCTREE_RESOLUTION = 0.2;

int verbosity_level_;

// Frames, topics and publishers
std::string laser_frame_, robot_frame_, odom_frame_, map_frame_;
std::string map_cloud_topic_, increment_cloud_topic_;
ros::Publisher map_cloud_pub_;

// tf handlers
tf::TransformListener* tf_listener_ptr_;
tf::TransformBroadcaster* tf_broadcaster_ptr_;

// PCL clouds for mapping
pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_(new pcl::PointCloud<pcl::PointXYZ>());
pcl::octree::OctreePointCloud<pcl::PointXYZ>::Ptr map_octree_(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION));


void transformCloudToFixedFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, std::string target_frame, std::string source_frame, ros::Time target_time, tf::TransformListener* tf_listener_ptr_)
{
    if(tf_listener_ptr_->canTransform(target_frame, source_frame, target_time))
        try
        {
            tf::StampedTransform transform;
            tf_listener_ptr_->lookupTransform(target_frame, source_frame, target_time, transform);
            pcl_ros::transformPointCloud(*cloud_in, *cloud_out, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    else
        ROS_INFO("Transform from %s to %s not available!", source_frame.c_str(), target_frame.c_str());
}


void initMap(std::string map_frame_, std::string odom_frame_, tf::TransformBroadcaster* tf_broadcaster_ptr_)
{
	map_octree_->setInputCloud(map_cloud_);
}


/* This function is inspired on https://github.com/erik-nelson/point_cloud_mapper */
void addPointsToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
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



#endif