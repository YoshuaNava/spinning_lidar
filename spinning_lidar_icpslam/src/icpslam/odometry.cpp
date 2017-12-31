
#include "icpslam/mapping.h"

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

// Constants for odometry
const double POSE_DIST_THRESH = 0.1;
const double ICP_FITNESS_THRESH = 0.1;
const double ICP_MAX_CORR_DIST = 1.0;
const double ICP_EPSILON = 1e-06;

int icp_max_iters;

int verbosity_level;

bool odom_inited;

// Input TF frames and topics
std::string laser_frame, robot_frame, odom_frame, map_frame;
std::string robot_odom_topic, robot_odom_path_topic, assembled_cloud_topic;

// ICP SLAM output topics and publishers
std::string map_cloud_topic, slam_pose_topic;
ros::Publisher robot_odom_path_pub, estimated_pose_pub, map_cloud_pub;


// ICP SLAM debug topics and publishers
std::string prev_cloud_topic, aligned_cloud_topic, icp_odom_topic, icp_odom_path_topic;
ros::Publisher prev_cloud_pub, aligned_cloud_pub, icp_odom_pub, icp_odom_path_pub;

// PCL clouds for odometry
pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZ>()), curr_cloud(new pcl::PointCloud<pcl::PointXYZ>());

// Robot/ICP odometry containers
nav_msgs::Path robot_odom_path, icp_odom_path;

// Translations and rotations estimated by ICP
std::vector<Eigen::Vector3f> icp_translations;
std::vector<Eigen::Quaternionf> icp_rotations;
tf::TransformListener* tf_listener_ptr;
tf::TransformBroadcaster* tf_broadcaster_ptr;


void robotOdometryCallback(const nav_msgs::Odometry::ConstPtr& robot_odom_msg)
{
	geometry_msgs::Pose curr_pose = robot_odom_msg->pose.pose;
	int num_poses = robot_odom_path.poses.size();
	
	if(num_poses > 0)
	{
		geometry_msgs::Pose prev_pose = robot_odom_path.poses[num_poses-1].pose;
		double dist = lengthOfVector(differenceBetweenPoses(prev_pose, curr_pose));
		if(dist < POSE_DIST_THRESH)
		{
			return;
		}
	}
	else
	{
		icp_translations.push_back(getTranslationFromROSPose(curr_pose));
		icp_rotations.push_back(getQuaternionFromROSPose(curr_pose));
		odom_inited = true;
		insertPoseInPath(curr_pose, odom_frame, robot_odom_msg->header.stamp, icp_odom_path);
	}

	if(verbosity_level >= 1)
	{
		std::cout << "Robot odometry position = " << getStringFromVector3f(getTranslationFromROSPose(curr_pose)) << std::endl;
		std::cout << "Robot odometry rotation = " << getStringFromQuaternion(getQuaternionFromROSPose(curr_pose)) << std::endl;
	}

	insertPoseInPath(curr_pose, odom_frame, robot_odom_msg->header.stamp, robot_odom_path);

	robot_odom_path.header.stamp = ros::Time().now();
	robot_odom_path.header.frame_id = odom_frame;
	robot_odom_path_pub.publish(robot_odom_path);
}



void publishOdometryPath()
{

}

void updateOdometry(Eigen::Matrix4f T)
{
	Eigen::Vector3f translation = getTranslationFromTMatrix(T);
	Eigen::Quaternionf rotation = getQuaternionFromTMatrix(T);

	int num_poses = icp_translations.size();

	Eigen::Vector3f prev_pos = icp_translations[num_poses-1];
	Eigen::Quaternionf prev_rot = icp_rotations[num_poses-1];
	Eigen::Vector3f curr_pos = prev_pos + prev_rot.toRotationMatrix() * translation;
	Eigen::Quaternionf curr_rot = prev_rot * rotation;
	curr_rot.normalize();

	// double dist = (prev_pos - curr_pos).norm();
	// if(dist < POSE_DIST_THRESH)
	{	
		icp_translations.push_back(curr_pos);
		icp_rotations.push_back(curr_rot);
		insertPoseInPath(curr_pos, curr_rot, odom_frame, ros::Time().now(), icp_odom_path);
		publishOdometry(curr_pos, curr_rot, odom_frame, robot_frame, ros::Time().now(), &icp_odom_pub);
		icp_odom_path.header.stamp = ros::Time().now();
		icp_odom_path.header.frame_id = odom_frame;
		icp_odom_path_pub.publish(icp_odom_path);
	}

	if(verbosity_level >= 1)
	{
		std::cout << "Initial position = " << getStringFromVector3f(icp_translations[0]) << std::endl;
		std::cout << "Initial rotation = " << getStringFromQuaternion(icp_rotations[0]) << std::endl;
		std::cout << "Cloud translation = " << getStringFromVector3f(translation) << std::endl;
		std::cout << "Cloud rotation = " << getStringFromQuaternion(rotation) << std::endl;
		std::cout << "ICP odometry position = " << getStringFromVector3f(curr_pos) << std::endl;
		std::cout << "ICP odometry rotation = " << getStringFromQuaternion(curr_rot) << std::endl;
		std::cout << std::endl;
	}
}


void assembledCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	ROS_INFO("Cloud callback!");
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*cloud_msg, *input_cloud);
	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
	voxel_filter.setInputCloud(input_cloud);
	voxel_filter.setLeafSize(0.05, 0.05, 0.05);
	voxel_filter.filter(*curr_cloud);

	if(prev_cloud->points.size() > 0)
	{
		// Registration
		// GICP is said to be better, but what about NICP from Serafin?
		pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(prev_cloud);
		icp.setInputTarget(curr_cloud);
		icp.setMaximumIterations(icp_max_iters);
		icp.setTransformationEpsilon(ICP_EPSILON);
		icp.setMaxCorrespondenceDistance(ICP_MAX_CORR_DIST);
		icp.setRANSACIterations(0);
		// icp.setMaximumOptimizerIterations(50);

		pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
		icp.align(*aligned_cloud);
		Eigen::Matrix4f T = icp.getFinalTransformation();
		
		if(icp.hasConverged() && odom_inited)
		{
			updateOdometry(T);

			// Publishing for debug
			if(verbosity_level >= 1)
			{
				ROS_INFO("# Cloud frame: %s", cloud_msg->header.frame_id.c_str());
				ROS_INFO("##   Registration");
				std::cout << "### ICP finished! \nConverged: " << icp.hasConverged() << " \nScore: " << icp.getFitnessScore() << std::endl;
				std::cout << T << std::endl;
				
				publishPointCloud(prev_cloud, cloud_msg->header.frame_id, ros::Time().now(), &prev_cloud_pub);
				publishPointCloud(aligned_cloud, cloud_msg->header.frame_id, ros::Time().now(), &aligned_cloud_pub);
			}
		}

	}

	*prev_cloud = *curr_cloud;
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
	nh.param("robot_odom_topic", robot_odom_topic, std::string("/odometry/filtered"));
	nh.param("robot_odom_path_topic", robot_odom_path_topic, std::string("robot_odom_path"));

	// ICP SLAM output topics
	nh.param("icp_max_iters", icp_max_iters, 10);
	nh.param("slam_pose_topic", slam_pose_topic, std::string("icpslam/pose"));
	nh.param("map_cloud_topic", map_cloud_topic, std::string("icpslam/map"));

	// ICP SLAM debug topics
	if(verbosity_level >=1)
	{
		nh.param("prev_cloud_topic", prev_cloud_topic, std::string("icpslam/prev_cloud"));
		nh.param("aligned_cloud_topic", aligned_cloud_topic, std::string("icpslam/aligned_cloud"));


		nh.param("icp_odom_topic", icp_odom_topic, std::string("icpslam/odom"));
		nh.param("icp_odom_path_topic", icp_odom_path_topic, std::string("icpslam/icp_odom_path"));
	}

	odom_inited = false;
}

void advertisePublishers(ros::NodeHandle nh)
{
	// ICP SLAM debug topics
	if(verbosity_level >=1)
	{
		prev_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(prev_cloud_topic, 1);
		aligned_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(aligned_cloud_topic, 1);

		icp_odom_pub = nh.advertise<nav_msgs::Odometry>(icp_odom_topic, 1);
		icp_odom_path_pub = nh.advertise<nav_msgs::Path>(icp_odom_path_topic, 1);
	}

	map_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(map_cloud_topic, 1);
	robot_odom_path_pub = nh.advertise<nav_msgs::Path>(robot_odom_path_topic, 1);
	estimated_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(slam_pose_topic, 1);
}


void registerSubscribers(ros::NodeHandle nh)
{

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "icpslam");
	ros::NodeHandle nh;

	loadParameters(nh);
	advertisePublishers(nh);
	
	tf::TransformListener tf_listener;
	tf_listener_ptr = &tf_listener;
	tf::TransformBroadcaster tf_broadcaster;
	tf_broadcaster_ptr = &tf_broadcaster;

	initMap(map_frame, odom_frame, tf_broadcaster_ptr);

	ros::Subscriber robot_odometry_sub = nh.subscribe(robot_odom_topic, 1, robotOdometryCallback);
	ros::Subscriber assembled_cloud_sub = nh.subscribe(assembled_cloud_topic, 1, assembledCloudCallback);

	ROS_INFO("ICP SLAM started");	

	ros::spin();
	

	return EXIT_SUCCESS;
}