
#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"
#include "utils/messaging_utils.h"
#include "icpslam/icp_odometer.h"

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

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>



ICPOdometer::ICPOdometer(ros::NodeHandle nh) :
	nh_(nh),
	prev_cloud_(new pcl::PointCloud<pcl::PointXYZ>()), curr_cloud_(new pcl::PointCloud<pcl::PointXYZ>())
{
	robot_odom_inited_ = false;
	new_transform_ = false;
	init();	
}

void ICPOdometer::init()
{
	loadParameters();
	advertisePublishers();
	registerSubscribers();

	ROS_INFO("ICP odometer initialized");
}

void ICPOdometer::loadParameters()
{
	nh_.param("verbosity_level_", verbosity_level_, 1);

	// TF frames
	nh_.param("map_frame", map_frame_, std::string("map"));
	nh_.param("odom_frame", odom_frame_, std::string("odom"));
	nh_.param("robot_frame", robot_frame_, std::string("base_link"));
	nh_.param("laser_frame", laser_frame_, std::string("laser"));

	// Input robot odometry and point cloud topics
	nh_.param("assembled_cloud_topic", assembled_cloud_topic_, std::string("spinning_lidar/assembled_cloud"));
	nh_.param("robot_odom_topic", robot_odom_topic_, std::string("/odometry/filtered"));
	nh_.param("robot_odom_path_topic", robot_odom_path_topic_, std::string("icpslam/robot_odom_path"));

	// ICP odometry debug topics
	if(verbosity_level_ >=1)
	{
		nh_.param("prev_cloud_topic", prev_cloud_topic_, std::string("icpslam/prev_cloud"));
		nh_.param("aligned_cloud_topic", aligned_cloud_topic_, std::string("icpslam/aligned_cloud"));

		nh_.param("icp_odom_topic", icp_odom_topic_, std::string("icpslam/odom"));
		nh_.param("icp_odom_path_topic", icp_odom_path_topic_, std::string("icpslam/icp_odom_path"));

		nh_.param("true_path_topic", true_path_topic_, std::string("icpslam/true_path"));
	}
}

void ICPOdometer::advertisePublishers()
{
	robot_odom_path_pub_ = nh_.advertise<nav_msgs::Path>(robot_odom_path_topic_, 1);
	
	// ICP odometry debug topics
	if(verbosity_level_ >=1)
	{
		prev_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(prev_cloud_topic_, 1);
		aligned_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(aligned_cloud_topic_, 1);

		icp_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(icp_odom_topic_, 1);
		icp_odom_path_pub_ = nh_.advertise<nav_msgs::Path>(icp_odom_path_topic_, 1);
		true_path_pub_ = nh_.advertise<nav_msgs::Path>(true_path_topic_, 1);
	}
}

void ICPOdometer::registerSubscribers()
{
	robot_odometry_sub_ = nh_.subscribe(robot_odom_topic_, 1, &ICPOdometer::robotOdometryCallback, this);
	assembled_cloud_sub_ = nh_.subscribe(assembled_cloud_topic_, 1, &ICPOdometer::assembledCloudCallback, this);
}

void ICPOdometer::publishInitialMapTransform(Pose6DOF map_in_robot)
{
	ROS_INFO("Map transform callback!");
	tf::Transform tf_map_in_odom;
	tf_map_in_odom.setOrigin( tf::Vector3(map_in_robot.pos(0), map_in_robot.pos(1), map_in_robot.pos(2)) );
    tf_map_in_odom.setRotation( tf::Quaternion(map_in_robot.rot.x(), map_in_robot.rot.y(), map_in_robot.rot.z(), map_in_robot.rot.w()) );
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_map_in_odom.inverse(), ros::Time::now(), odom_frame_, map_frame_));
}

void ICPOdometer::publishDebugTransform(Pose6DOF frame_in_robot)
{
	// ROS_INFO("Map transform callback!");
	tf::Transform tf_frame_in_robot;
	tf_frame_in_robot.setOrigin( tf::Vector3(frame_in_robot.pos(0), frame_in_robot.pos(1), frame_in_robot.pos(2)) );
    tf_frame_in_robot.setRotation( tf::Quaternion(frame_in_robot.rot.x(), frame_in_robot.rot.y(), frame_in_robot.rot.z(), frame_in_robot.rot.w()) );
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_frame_in_robot.inverse(), ros::Time::now(), odom_frame_, "debug_frame_2"));
}

bool ICPOdometer::isOdomReady()
{
	return robot_odom_inited_;
}

Pose6DOF ICPOdometer::getFirstPoseRobotOdometry()
{
	return robot_odom_poses_.front(); 
}

Pose6DOF ICPOdometer::getLatestPoseRobotOdometry() 
{
	return robot_odom_poses_.back(); 
}

Pose6DOF ICPOdometer::getLatestPoseICPOdometry() 
{
	return icp_odom_poses_.back();
}

void ICPOdometer::getLatestCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, Pose6DOF *latest_transform, Pose6DOF *icp_pose, Pose6DOF *odom_pose, bool *new_transform)
{
	**cloud = *prev_cloud_;
	*latest_transform = icp_latest_transform_;

	if(icp_odom_poses_.size() > 0)
		*icp_pose = icp_odom_poses_.back();
	else
		*icp_pose = robot_odom_poses_.back();

	*odom_pose = robot_odom_poses_.back();
	*new_transform = new_transform_;

	this->new_transform_ = false;
}

void ICPOdometer::robotOdometryCallback(const nav_msgs::Odometry::ConstPtr& robot_odom_msg)
{
	// ROS_INFO("Robot odometry callback!");
	geometry_msgs::PoseWithCovariance pose_cov_msg = robot_odom_msg->pose;
	Pose6DOF pose_in_odom(pose_cov_msg, robot_odom_msg->header.stamp), 
			 pose_in_map;

	if(tf_listener_.canTransform(map_frame_, odom_frame_, ros::Time(0)))
		pose_in_map = Pose6DOF::transformToFixedFrame(pose_in_odom, map_frame_, odom_frame_, &tf_listener_);
	else
		return;

	if(robot_odom_poses_.size() == 0)
	{
		// rodom_first_pose.setIdentity();
		rodom_first_pose = pose_in_map;
	}
	
	Pose6DOF origin_diff = Pose6DOF::subtract(pose_in_map, rodom_first_pose);
	publishDebugTransform(origin_diff);
	
	robot_odom_poses_.push_back(pose_in_map);

	int num_poses = robot_odom_path_.poses.size();
	if(num_poses > 0)
	{
		Pose6DOF prev_pose(robot_odom_path_.poses[num_poses-1].pose);
		if(Pose6DOF::distanceEuclidean(pose_in_map, prev_pose) < POSE_DIST_THRESH)
		{
			return;
		}
	}
	else
	{
		icp_odom_poses_.push_back(pose_in_map);
		robot_odom_inited_ = true;
		insertPoseInPath(pose_in_map.toROSPose(), map_frame_, robot_odom_msg->header.stamp, icp_odom_path_);
	}

	insertPoseInPath(pose_in_map.toROSPose(), map_frame_, robot_odom_msg->header.stamp, robot_odom_path_);
	robot_odom_path_.header.stamp = ros::Time().now();
	robot_odom_path_.header.frame_id = map_frame_;
	robot_odom_path_pub_.publish(robot_odom_path_);
	if( tf_listener_.canTransform(odom_frame_, map_frame_, ros::Time(0)) )
	{
		insertPoseInPath(pose_in_map.toROSPose(), map_frame_, robot_odom_msg->header.stamp, true_path_);
		true_path_.header.stamp = ros::Time().now();
		true_path_.header.frame_id = map_frame_;
		true_path_pub_.publish(true_path_);
	}

	if(verbosity_level_ >= 2)
	{
		std::cout << "Robot odometry says: " << pose_in_map.toStringQuat("   ");
		std::cout << "In map, robot odometry says: " << pose_in_map.toStringQuat("   ");
		std::cout << std::endl;
	}
}

void ICPOdometer::updateICPOdometry(Eigen::Matrix4d T)
{
	// ROS_INFO("ICP odometry update!");
	new_transform_ = true;
	Pose6DOF transform_in_odom(T, ros::Time().now());
	// transform_in_odom.pos *= -1.0;
	icp_latest_transform_ = transform_in_odom;

	Pose6DOF prev_pose = getLatestPoseICPOdometry();
	Pose6DOF new_pose = Pose6DOF::compose(prev_pose, transform_in_odom);
	new_pose.time_stamp = ros::Time().now();

	icp_odom_poses_.push_back(new_pose);
	publishOdometry(new_pose.pos, new_pose.rot, map_frame_, odom_frame_, ros::Time().now(), &icp_odom_pub_);
	insertPoseInPath(new_pose.toROSPose(), map_frame_, ros::Time().now(), icp_odom_path_);
	icp_odom_path_.header.stamp = ros::Time().now();
	icp_odom_path_.header.frame_id = map_frame_;
	icp_odom_path_pub_.publish(icp_odom_path_);

	if(verbosity_level_ >= 2)
	{
		std::cout << std::endl;
		std::cout << "Initial position = " << getStringFromVector3d(icp_odom_poses_[0].pos) << std::endl;
		std::cout << "Initial rotation = " << getStringFromQuaternion(icp_odom_poses_[0].rot) << std::endl;
		std::cout << "Prev odom pose = \n" << prev_pose;
		std::cout << "Cloud translation = " << getStringFromVector3d(transform_in_odom.pos) << std::endl;
		std::cout << "Cloud rotation = " << getStringFromQuaternion(transform_in_odom.rot) << std::endl;
		std::cout << "ICP odometry position = " << getStringFromVector3d(new_pose.pos) << std::endl;
		std::cout << "ICP odometry rotation = " << getStringFromQuaternion(new_pose.rot) << std::endl;
		std::cout << std::endl;
	}
}

void ICPOdometer::assembledCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	// ROS_INFO("Cloud callback!");
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*cloud_msg, *input_cloud);
	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
	voxel_filter.setInputCloud(input_cloud);
	voxel_filter.setLeafSize(0.05, 0.05, 0.05);
	voxel_filter.filter(*curr_cloud_);

	if(prev_cloud_->points.size() > 0 && robot_odom_inited_)
	{
		// Registration
		// GICP is said to be better, but what about NICP from Serafin?
		pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(curr_cloud_);
		icp.setInputTarget(prev_cloud_);
		icp.setMaximumIterations(ICP_MAX_ITERS);
		icp.setTransformationEpsilon(ICP_EPSILON);
		icp.setMaxCorrespondenceDistance(ICP_MAX_CORR_DIST);
		icp.setRANSACIterations(0);

		pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
		icp.align(*aligned_cloud);
		Eigen::Matrix4d T = icp.getFinalTransformation().cast<double>();
		
		if(icp.hasConverged())
		{
			updateICPOdometry(T);

			// Publishing for debug
			if(verbosity_level_ >= 1)
			{				
				publishPointCloud(prev_cloud_, cloud_msg->header.frame_id, ros::Time().now(), &prev_cloud_pub_);
				publishPointCloud(aligned_cloud, cloud_msg->header.frame_id, ros::Time().now(), &aligned_cloud_pub_);
			}
		}

	}
	
	*prev_cloud_ = *curr_cloud_;
}
