
#ifndef GEOMETRIC_UTILS_H
#define GEOMETRIC_UTILS_H

#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct Pose6DOF
{
	Eigen::Vector3d pos;
	Eigen::Quaterniond rot;
	Eigen::Matrix<double, 6, 6> cov;
};

Eigen::Matrix<double, 6, 6> getCovarianceFromROSPoseWithCovariance(geometry_msgs::PoseWithCovariance pose_msg);

tf::Transform getTFTransformFromROSOdometry(nav_msgs::Odometry odom_msg);

tf::Transform getInverseTFTransformFromROSOdometry(nav_msgs::Odometry odom_msg);

tf::Transform getTFTransformFromPositionQuaternion(Eigen::Vector3d pos, Eigen::Quaterniond q);

tf::Pose getTFPoseFromPositionQuaternion(Eigen::Vector3d pos, Eigen::Quaterniond q);

Eigen::Vector3d getTranslationFromTMatrix(Eigen::Matrix4f &T);

Eigen::Quaterniond getQuaternionFromTMatrix(Eigen::Matrix4f &T);

geometry_msgs::Pose getROSPoseFromPosQuat(Eigen::Vector3d pos, Eigen::Quaterniond q);

Eigen::Vector3d getTranslationFromROSPose(geometry_msgs::Pose pose);

Eigen::Quaterniond getQuaternionFromROSPose(geometry_msgs::Pose pose);

std::string getStringFromVector3d(Eigen::Vector3d vector);

std::string getStringFromQuaternion(Eigen::Quaterniond q);

tf::Pose differenceBetweenPoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2);

double lengthOfVector(tf::Pose vector);

#endif