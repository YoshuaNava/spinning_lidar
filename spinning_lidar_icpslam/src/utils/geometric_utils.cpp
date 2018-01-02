

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utils/geometric_utils.h"


geometry_msgs::Point getROSPointFromPose6DOF(Pose6DOF pose)
{
	geometry_msgs::Point p;
	p.x = pose.pos(0);
	p.y = pose.pos(1);
	p.z = pose.pos(2);

	return p;
}

// Reference: https://github.com/ethz-asl/ethzasl_msf/blob/master/msf_eval/src/msf_eval.cpp
Eigen::Matrix<double, 6, 6> getCovarianceFromROSPoseWithCovariance(geometry_msgs::PoseWithCovariance pose_msg)
{
	double* cov_arr = pose_msg.covariance.data();
	return Eigen::Matrix<double, 6, 6>(cov_arr);
}

tf::Transform getTFTransformFromROSOdometry(nav_msgs::Odometry odom_msg)
{
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z));
	transform.setRotation(tf::Quaternion(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w));

	return transform;
}

tf::Transform getInverseTFTransformFromROSOdometry(nav_msgs::Odometry odom_msg)
{
	return getTFTransformFromROSOdometry(odom_msg).inverse();
}

tf::Transform getTFTransformFromPositionQuaternion(Eigen::Vector3d pos, Eigen::Quaterniond q)
{
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(pos(0), pos(1), pos(2)));
	transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
	return transform;
}

tf::Pose getTFPoseFromPositionQuaternion(Eigen::Vector3d pos, Eigen::Quaterniond q)
{
	tf::Pose pose;
	pose.setOrigin(tf::Vector3(pos(0), pos(1), pos(2)));
	pose.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
	return pose;
}

Eigen::Vector3d getTranslationFromTMatrix(Eigen::Matrix4f &T)
{
	Eigen::Vector3d translation(-T(0,3), -T(1,3), -T(2,3));
	return translation;
}

Eigen::Quaterniond getQuaternionFromTMatrix(Eigen::Matrix4f &T)
{
	Eigen::Matrix3d rot = T.block(0, 0, 3, 3).transpose().cast<double>();
	Eigen::Quaterniond q(rot);
	return q;
}

geometry_msgs::Pose getROSPoseFromPosQuat(Eigen::Vector3d pos, Eigen::Quaterniond q)
{
	geometry_msgs::Pose pose;
	pose.position.x = pos(0);
	pose.position.y = pos(1);
	pose.position.z = pos(2);
	pose.orientation.x = q.x();
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
	pose.orientation.w = q.w();
	return pose;
}

Eigen::Vector3d getTranslationFromROSPose(geometry_msgs::Pose pose)
{
	Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);
	return translation;
}

Eigen::Quaterniond getQuaternionFromROSPose(geometry_msgs::Pose pose)
{
	Eigen::Quaterniond q;
	q.x() = pose.orientation.x;
	q.y() = pose.orientation.y;
	q.z() = pose.orientation.z;
	q.w() = pose.orientation.w;
	return q;
}


std::string getStringFromVector3d(Eigen::Vector3d vector)
{
	std::string output = "(";
	output += std::to_string(vector(0)) + ", ";
    output += std::to_string(vector(1)) + ", ";
    output += std::to_string(vector(2)) + ") with Norm = ";
	output += std::to_string(vector.norm());
	return output;
}

std::string getStringFromQuaternion(Eigen::Quaterniond q)
{
	std::string output = "(";
	output += std::to_string(q.x()) + ", ";
	output += std::to_string(q.y()) + ", ";
	output += std::to_string(q.z()) + ", ";
	output += std::to_string(q.w()) + ") with Norm = ";
	output += std::to_string(q.norm());
	return output;
}


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
