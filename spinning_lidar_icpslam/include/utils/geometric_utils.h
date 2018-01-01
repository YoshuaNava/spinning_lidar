
#ifndef GEOMETRIC_UTILS_H
#define GEOMETRIC_UTILS_H

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct Pose6DOF
{
	Eigen::Vector3f pos;
	Eigen::Quaternionf rot;
};


tf::Transform getTFTransformFromROSOdometry(nav_msgs::Odometry odom_msg);

tf::Transform getInverseTFTransformFromROSOdometry(nav_msgs::Odometry odom_msg);

tf::Transform getTFTransformFromPositionQuaternion(Eigen::Vector3f pos, Eigen::Quaternionf q);

tf::Pose getTFPoseFromPositionQuaternion(Eigen::Vector3f pos, Eigen::Quaternionf q);

Eigen::Vector3f getTranslationFromTMatrix(Eigen::Matrix4f &T);

Eigen::Quaternionf getQuaternionFromTMatrix(Eigen::Matrix4f &T);

geometry_msgs::Pose getROSPoseFromPosQuat(Eigen::Vector3f pos, Eigen::Quaternionf q);

Eigen::Vector3f getTranslationFromROSPose(geometry_msgs::Pose pose);

Eigen::Quaternionf getQuaternionFromROSPose(geometry_msgs::Pose pose);

std::string getStringFromVector3f(Eigen::Vector3f vector);

std::string getStringFromQuaternion(Eigen::Quaternionf q);

tf::Pose differenceBetweenPoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2);

double lengthOfVector(tf::Pose vector);

#endif