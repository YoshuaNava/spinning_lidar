
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "geometric_utils.h"

const double POSE_DIST_THRESH = 0.05;
const double ICP_FITNESS_THRESH = 0.1;


bool odom_inited;
double icp_epsilon;
int icp_max_iters;

int verbosity_level;

// Input TF links and topics
std::string laser_link, robot_link, odom_link;
std::string robot_odom_topic, robot_odom_path_topic, assembled_cloud_topic;

// ICP SLAM output topics and publishers
std::string map_cloud_topic, icp_odom_path_topic, slam_pose_topic;
ros::Publisher robot_odom_path_pub, estimated_pose_pub, map_cloud_pub;

// ICP SLAM debug topics and publishers
std::string prev_cloud_topic, aligned_cloud_topic;
ros::Publisher prev_cloud_pub, aligned_cloud_pub, icp_odom_path_pub;


pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZ>()), curr_cloud(new pcl::PointCloud<pcl::PointXYZ>()), map_cloud(new pcl::PointCloud<pcl::PointXYZ>());

nav_msgs::Odometry last_robot_odom_msg;
nav_msgs::Path robot_odom_path, icp_odom_path;

std::vector<Eigen::Vector3f> icp_translations;
std::vector<Eigen::Quaternionf> icp_rotations;
tf::TransformListener* tf_listener_ptr;


void publishMapCloud()
{
	try
	{
		tf::StampedTransform transform;
		tf_listener_ptr->lookupTransform(odom_link, robot_link, ros::Time(0), transform);
		pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_odom_frame(new pcl::PointCloud<pcl::PointXYZ>());
		pcl_ros::transformPointCloud(*map_cloud, *map_cloud_odom_frame, transform);
		sensor_msgs::PointCloud2 map_cloud_msg;
		pcl::toROSMsg(*map_cloud, map_cloud_msg);
		map_cloud_msg.header.stamp = ros::Time().now();
		map_cloud_msg.header.frame_id = odom_link;
		map_cloud_pub.publish(map_cloud_msg);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
	}
}


void robotOdometryCallback(const nav_msgs::Odometry::ConstPtr& robot_odom_msg)
{
	last_robot_odom_msg = *robot_odom_msg;

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
	}

	geometry_msgs::PoseStamped pose_stamped_msg;
	pose_stamped_msg.pose = curr_pose;
	pose_stamped_msg.header.stamp = last_robot_odom_msg.header.stamp;
	pose_stamped_msg.header.frame_id = odom_link;
	robot_odom_path.header.stamp = ros::Time().now();
	robot_odom_path.header.frame_id = odom_link;
	robot_odom_path.poses.push_back(pose_stamped_msg);

	if(icp_odom_path.poses.size() == 0)
		icp_odom_path.poses.push_back(pose_stamped_msg);

	robot_odom_path_pub.publish(robot_odom_path);
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
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(prev_cloud);
		icp.setInputTarget(curr_cloud);
		icp.setMaximumIterations(icp_max_iters);
		icp.setTransformationEpsilon(icp_epsilon);
		pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
		icp.align(*aligned_cloud);
		Eigen::Matrix4f T = icp.getFinalTransformation();
		
		if((icp.getFitnessScore() <= ICP_FITNESS_THRESH) && icp.hasConverged() && odom_inited)
		{
			Eigen::Vector3f translation = getTranslationFromTMatrix(T);
			Eigen::Quaternionf rotation = getQuaternionFromTMatrix(T);

			int num_poses = icp_translations.size();
			Eigen::Vector3f prev_position = icp_translations[num_poses-1];
			Eigen::Quaternionf prev_orientation = icp_rotations[num_poses-1];

			// Eigen::Vector3f curr_position = icp_translations[0] + translation;
			// Eigen::Quaternionf curr_orientation = icp_rotations[0] * rotation;
			Eigen::Vector3f curr_position = prev_position + translation;
			Eigen::Quaternionf curr_orientation = prev_orientation * rotation;
			icp_translations.push_back(curr_position);
			icp_rotations.push_back(curr_orientation);

			if(map_cloud->points.size() == 0)
			{
				*map_cloud = *aligned_cloud;
				publishMapCloud();
			}
				

			// Publishing for debug
			if(verbosity_level >= 1)
			{
				ROS_INFO("#   Registration");
				std::cout << "## ICP finished! \nConverged: " << icp.hasConverged() << " \nScore: " << icp.getFitnessScore() << std::endl;
				std::cout << T << std::endl;
				sensor_msgs::PointCloud2 prev_cloud_msg;
				pcl::toROSMsg(*prev_cloud, prev_cloud_msg);
				prev_cloud_msg.header.stamp = ros::Time().now();
				prev_cloud_msg.header.frame_id = cloud_msg->header.frame_id;
				prev_cloud_pub.publish(prev_cloud_msg);

				sensor_msgs::PointCloud2 aligned_cloud_msg;
				pcl::toROSMsg(*aligned_cloud, aligned_cloud_msg);
				aligned_cloud_msg.header.stamp = ros::Time().now();
				aligned_cloud_msg.header.frame_id = cloud_msg->header.frame_id;
				aligned_cloud_pub.publish(aligned_cloud_msg);
				geometry_msgs::PoseStamped pose_stamped_msg;
				pose_stamped_msg.pose = getROSPoseFromPosQuat(curr_position, curr_orientation);
				pose_stamped_msg.header.stamp = ros::Time().now();
				pose_stamped_msg.header.frame_id = odom_link;
				icp_odom_path.header.stamp = ros::Time().now();
				icp_odom_path.header.frame_id = odom_link;
				icp_odom_path.poses.push_back(pose_stamped_msg);
				icp_odom_path_pub.publish(icp_odom_path);

				std::cout << "Initial position = " << getStringFromVector3f(icp_translations[0]) << std::endl;
				std::cout << "Initial rotation = " << getStringFromQuaternion(icp_rotations[0]) << std::endl;
				std::cout << "Cloud translation = " << getStringFromVector3f(translation) << std::endl;
				std::cout << "Cloud rotation = " << getStringFromQuaternion(rotation) << std::endl;
				std::cout << "Current position = " << getStringFromVector3f(curr_position) << std::endl;
				std::cout << "Current rotation = " << getStringFromQuaternion(curr_orientation) << std::endl;
				std::cout << std::endl;
			}
		}

	}
	// else
	// {
	// 	std::cerr << "Re-setting prev_cloud" << std::endl;
	// 	*prev_cloud = *curr_cloud;
	// }
	*prev_cloud = *curr_cloud;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "icpslam");
	ros::NodeHandle nh;

	nh.param("verbosity_level", verbosity_level, 1);

	// TF links
	nh.param("odom_link", odom_link, std::string("odom"));
	nh.param("robot_link", robot_link, std::string("base_link"));
	nh.param("laser_link", laser_link, std::string("laser"));

	// Input robot odometry and point cloud topics
    nh.param("assembled_cloud_topic", assembled_cloud_topic, std::string("spinning_lidar/assembled_cloud"));
	nh.param("robot_odom_topic", robot_odom_topic, std::string("/odometry/filtered"));
	nh.param("robot_odom_path_topic", robot_odom_path_topic, std::string("robot_odom_path"));

	// ICP SLAM output topics
	nh.param("icp_epsilon", icp_epsilon, 1e-06);
	nh.param("icp_max_iters", icp_max_iters, 10);
	nh.param("slam_pose_topic", slam_pose_topic, std::string("icpslam/pose"));
	nh.param("map_cloud_topic", map_cloud_topic, std::string("icpslam/map"));

	// ICP SLAM debug topics
	if(verbosity_level >=1)
	{
		nh.param("icp_odom_path_topic", icp_odom_path_topic, std::string("icpslam/icp_odom_path"));
		nh.param("prev_cloud_topic", prev_cloud_topic, std::string("icpslam/prev_cloud"));
		nh.param("aligned_cloud_topic", aligned_cloud_topic, std::string("icpslam/aligned_cloud"));
		prev_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(prev_cloud_topic, 1);
		aligned_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(aligned_cloud_topic, 1);
		icp_odom_path_pub = nh.advertise<nav_msgs::Path>(icp_odom_path_topic, 1);
	}

	map_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(map_cloud_topic, 1);
	robot_odom_path_pub = nh.advertise<nav_msgs::Path>(robot_odom_path_topic, 1);
	estimated_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(slam_pose_topic, 1);

	odom_inited = false;

	ros::Subscriber robot_odometry_sub = nh.subscribe(robot_odom_topic, 1, robotOdometryCallback);
	ros::Subscriber assembled_cloud_sub = nh.subscribe(assembled_cloud_topic, 1, assembledCloudCallback);
	
	tf::TransformListener tf_listener;
	tf_listener_ptr = &tf_listener;

	ROS_INFO("ICP SLAM started");
	ROS_INFO("Listening to robot odometry messages at %s", robot_odom_topic.c_str());
	ROS_INFO("Listening to assembled cloud messages at %s", assembled_cloud_topic.c_str());
	

	ros::spin();
	

	return EXIT_SUCCESS;
}