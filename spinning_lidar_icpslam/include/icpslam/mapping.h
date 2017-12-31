
#ifndef MAPPING_H
#define MAPPING_H


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// General
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "icpslam/geometric_utils.h"
#include "icpslam/messaging_utils.h"


// Odometry
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "icpslam/geometric_utils.h"
#include "icpslam/messaging_utils.h"

// Mapping
#include <pcl/octree/octree_search.h>




// Constants for mapping
const float OCTREE_RESOLUTION = 0.2;
// PCL clouds for mapping
pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::octree::OctreePointCloud<pcl::PointXYZ>::Ptr map_octree(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION));


void transformCloudToFixedFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, std::string target_frame, std::string source_frame, ros::Time target_time, tf::TransformListener* tf_listener_ptr)
{
    if(tf_listener_ptr->canTransform(target_frame, source_frame, target_time))
        try
        {
            tf::StampedTransform transform;
            tf_listener_ptr->lookupTransform(target_frame, source_frame, target_time, transform);
            pcl_ros::transformPointCloud(*cloud_in, *cloud_out, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    else
        ROS_INFO("Transform from %s to %s not available!", source_frame.c_str(), target_frame.c_str());
}


void initMap(std::string map_frame, std::string odom_frame, tf::TransformBroadcaster* tf_broadcaster_ptr)
{
    // Publish transform between odom and map for the first time
    // tf::Transform transform;
    // transform.setOrigin( tf::Vector3(0, 0, 0) );
    // transform.setRotation( tf::Quaternion(0, 0, 0, 1) );        // Angle = 0 degrees
    // tf_broadcaster_ptr->sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame, odom_frame));

	map_octree->setInputCloud(map_cloud);
}


/* This function is inspired on https://github.com/erik-nelson/point_cloud_mapper */
void addPointsToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	for(size_t i=0; i<input_cloud->points.size() ;i++)
	{
		pcl::PointXYZ point = input_cloud->points[i];
		if(!map_octree->isVoxelOccupiedAtPoint(point))
		{
			map_octree->addPointToCloud(point, map_cloud);
		}
	}
}



#endif