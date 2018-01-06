
#ifndef MESSAGING_UTILS_H
#define MESSAGING_UTILS_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"


void insertPoseInPath(Eigen::Vector3d position, Eigen::Quaterniond orientation, std::string frame_id, ros::Time stamp, nav_msgs::Path &path);

void insertPoseInPath(geometry_msgs::Pose pose, std::string frame_id, ros::Time stamp, nav_msgs::Path &path);

void publishOdometry(Eigen::Vector3d position, Eigen::Quaterniond orientation, std::string ref_frame, std::string robot_frame, ros::Time stamp, ros::Publisher* pub_ptr);

void publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string frame_id, ros::Time stamp, ros::Publisher* pub_ptr);

#endif