
#ifndef ICP_ODOMETER_H
#define ICP_ODOMETER_H

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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"


class ICPOdometer
{
private:
	// Constants
	typedef ros::Time TimeStamp;
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

	const double POSE_DIST_THRESH = 0.1;
	const double ICP_FITNESS_THRESH = 0.1;
	const double ICP_MAX_CORR_DIST = 1.0;
	const double ICP_EPSILON = 1e-06;
	const double ICP_MAX_ITERS = 10;

	int verbosity_level_;

	bool robot_odom_inited_;

	// ROS node handle, URDF frames, topics and publishers
	ros::NodeHandle nh_;
	std::string laser_frame_, robot_frame_, odom_frame_, map_frame_;
	std::string robot_odom_topic_, robot_odom_path_topic_, assembled_cloud_topic_;
	ros::Publisher robot_odom_path_pub_;
	ros::Subscriber robot_odometry_sub_, assembled_cloud_sub_;

	// ICP odometry debug topics and publishers
	std::string prev_cloud_topic_, aligned_cloud_topic_, icp_odom_topic_, icp_odom_path_topic_, true_path_topic_;
	ros::Publisher prev_cloud_pub_, aligned_cloud_pub_, icp_odom_pub_, icp_odom_path_pub_, true_path_pub_;

	// PCL clouds for odometry
	pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_, curr_cloud_;

	// ICP odometry containers
	nav_msgs::Path robot_odom_path_, icp_odom_path_, true_path_;

	// Translations and rotations estimated by ICP
	bool new_transform_;
	Pose6DOF rodom_first_pose, icp_latest_transform_;
	std::vector<Pose6DOF> icp_odom_poses_;
	std::vector<Pose6DOF> robot_odom_poses_;
	tf::TransformListener tf_listener_;
	tf::TransformBroadcaster tf_broadcaster_;

public:

	ICPOdometer(ros::NodeHandle nh);

	void init();

	void loadParameters();

	void advertisePublishers();

	void registerSubscribers();

	void publishInitialMapTransform(Pose6DOF map_in_robot = Pose6DOF::getIdentity());
	
	void publishDebugTransform(Pose6DOF frame_in_robot);

	bool isOdomReady();

	Pose6DOF getFirstPoseRobotOdometry();

 	Pose6DOF getLatestPoseRobotOdometry(); 
	
	Pose6DOF getLatestPoseICPOdometry();

	void getLatestCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, Pose6DOF *latest_transform, Pose6DOF *icp_pose, Pose6DOF *odom_pose, bool *new_transform);

	void robotOdometryCallback(const nav_msgs::Odometry::ConstPtr& robot_odom_msg);

	void updateICPOdometry(Eigen::Matrix4d T);

	void assembledCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
};

#endif