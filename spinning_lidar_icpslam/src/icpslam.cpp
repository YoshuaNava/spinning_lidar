
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

const double POSE_DIST_THRESH = 0.1;

std::string laser_link, odom_link;
std::string assembled_cloud_topic, map_cloud_topic, odometry_topic, path_topic, pose_topic;

nav_msgs::Odometry last_odom_msg;
nav_msgs::Path odom_path_msg;

pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZ>()), curr_cloud(new pcl::PointCloud<pcl::PointXYZ>()), map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
ros::Publisher odom_path_pub, estimated_pose_pub, map_cloud_pub;

tf::Pose differenceBetweenPoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
	tf::Pose tf1, tf2;
	tf1.setOrigin(tf::Vector3(p1.position.x, p1.position.y, p1.position.z));
	tf1.setRotation(tf::Quaternion(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w));
	tf2.setOrigin(tf::Vector3(p2.position.x, p2.position.y, p2.position.z));
	tf2.setRotation(tf::Quaternion(p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w));

	return tf1.inverseTimes(tf2);
}

double lengthOfVector(tf::Pose vector)
{
	return vector.getOrigin().length();
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	last_odom_msg = *odom_msg;

	geometry_msgs::Pose curr_pose = odom_msg->pose.pose;
	int num_poses = odom_path_msg.poses.size();

	if(num_poses > 0)
	{
		geometry_msgs::Pose prev_pose = odom_path_msg.poses[num_poses-1].pose;
		
		double dist = lengthOfVector(differenceBetweenPoses(prev_pose, curr_pose));

		if(dist < POSE_DIST_THRESH)
		{
			return;
		}
	}

	geometry_msgs::PoseStamped pose_stamped_msg;
	pose_stamped_msg.pose = curr_pose;
	pose_stamped_msg.header.stamp = odom_msg->header.stamp;
	pose_stamped_msg.header.frame_id = odom_link;
	odom_path_msg.header.stamp = ros::Time().now();
	odom_path_msg.header.frame_id = odom_link;
	odom_path_msg.poses.push_back(pose_stamped_msg);
	odom_path_pub.publish(odom_path_msg);
}


void assembledCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	ROS_INFO("Cloud callback!");
	pcl::fromROSMsg(*cloud_msg, *curr_cloud);
	if(prev_cloud != NULL)
	{
		if(prev_cloud->points.size() > 0)
		{
			// Registration
			ROS_INFO("####   Registration");
			pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
			int max_iters = 30;
			icp.setInputSource(prev_cloud);
			icp.setInputTarget(curr_cloud);
			icp.setMaximumIterations(max_iters);
			pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
			icp.align(*registered_cloud);
			std::cout << "# ICP finished! \nConverged: " << icp.hasConverged() << " \nScore: " <<
			icp.getFitnessScore() << std::endl;
			std::cout << icp.getFinalTransformation() << std::endl;

			pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
			voxel_filter.setInputCloud(registered_cloud);
			voxel_filter.setLeafSize(0.05, 0.05, 0.05);
			voxel_filter.filter(*map_cloud);

			sensor_msgs::PointCloud2 map_cloud_msg;
			pcl::toROSMsg(*registered_cloud, map_cloud_msg);
			map_cloud_msg.header.stamp = ros::Time().now();
			map_cloud_msg.header.stamp = ros::Time().now();
			map_cloud_msg.header.frame_id = "odom";
			map_cloud_pub.publish(map_cloud_msg);

			prev_cloud = map_cloud;
		}
		else
		{
			prev_cloud = curr_cloud;
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "icpslam");
	ros::NodeHandle nh;

	nh.param("odom_link", odom_link, std::string("odom"));
	nh.param("laser_link", laser_link, std::string("laser"));
    nh.param("assembled_cloud_topic", assembled_cloud_topic, std::string("spinning_lidar/assembled_cloud"));
    nh.param("map_cloud_topic", map_cloud_topic, std::string("icpslam/map"));
	nh.param("odometry_topic", odometry_topic, std::string("/odometry/filtered"));
	nh.param("path_topic", path_topic, std::string("icpslam/path"));
	nh.param("pose_topic", pose_topic, std::string("icpslam/pose"));

	map_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(map_cloud_topic, 1);
	odom_path_pub = nh.advertise<nav_msgs::Path>(path_topic, 1);
	estimated_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 1);

	ros::Subscriber odometry_sub = nh.subscribe(odometry_topic, 1, odometryCallback);
	ros::Subscriber assembled_cloud_sub = nh.subscribe(assembled_cloud_topic, 1, assembledCloudCallback);

	ROS_INFO("ICP SLAM started");
	ROS_INFO("Listening to odometry messages at %s", odometry_topic.c_str());
	ROS_INFO("Listening to assembled cloud messages at %s", assembled_cloud_topic.c_str());
		
	ros::spin();

	return EXIT_SUCCESS;
}