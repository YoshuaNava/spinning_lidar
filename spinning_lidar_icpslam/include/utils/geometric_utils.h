
#ifndef GEOMETRIC_UTILS_H
#define GEOMETRIC_UTILS_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Pose6DOF
{
public:
	ros::Time time_stamp;
	Eigen::Vector3d pos;
	Eigen::Quaterniond rot;
	Eigen::Matrix<double, 6, 6> cov;

	Pose6DOF()
	{
		pos = Eigen::Vector3d(0,0,0);
		rot = Eigen::Quaterniond(0,0,0,1);
		cov = Eigen::MatrixXd::Zero(6,6);
	}

	Pose6DOF(Eigen::Matrix4d &T)
	{
		this->fromTMatrix(T);
	}

	Pose6DOF& operator =(Pose6DOF pose)
	{
		this->time_stamp = pose.time_stamp;
		this->pos(0) = pose.pos(0);
		this->pos(1) = pose.pos(1);
		this->pos(2) = pose.pos(2);

		this->rot.x() = pose.rot.x();
		this->rot.y() = pose.rot.y();
		this->rot.z() = pose.rot.z();
		this->rot.w() = pose.rot.w();
		this->cov = pose.cov;
		
		return *this;
	}

	bool operator ==(const Pose6DOF &pose)
	{
		return (((this->pos - pose.pos).norm() < EQUALITY_THRESH) && (fabs(this->rot.dot(pose.rot)) < 1-EQUALITY_THRESH));
	}

	void fromTMatrix(Eigen::Matrix4d &T)
	{
		this->pos = Eigen::Vector3d(-T(0,3), -T(1,3), -T(2,3));
		Eigen::Matrix3d rot = T.block(0, 0, 3, 3).transpose().cast<double>();
		this->rot = Eigen::Quaterniond(rot);
	}


private:
	const double EQUALITY_THRESH = 1e-10;
};


geometry_msgs::Point getROSPointFromPose6DOF(Pose6DOF pose);

Eigen::Matrix<double, 6, 6> getCovarianceFromROSPoseWithCovariance(geometry_msgs::PoseWithCovariance pose_msg);

tf::Transform getTFTransformFromROSOdometry(nav_msgs::Odometry odom_msg);

tf::Transform getInverseTFTransformFromROSOdometry(nav_msgs::Odometry odom_msg);

tf::Transform getTFTransformFromPositionQuaternion(Eigen::Vector3d pos, Eigen::Quaterniond q);

tf::Pose getTFPoseFromPositionQuaternion(Eigen::Vector3d pos, Eigen::Quaterniond q);

Eigen::Vector3d getTranslationFromTMatrix(Eigen::Matrix4d &T);

Eigen::Quaterniond getQuaternionFromTMatrix(Eigen::Matrix4d &T);

geometry_msgs::Pose getROSPoseFromPosQuat(Eigen::Vector3d pos, Eigen::Quaterniond q);

Eigen::Vector3d getTranslationFromROSPose(geometry_msgs::Pose pose);

Eigen::Quaterniond getQuaternionFromROSPose(geometry_msgs::Pose pose);

std::string getStringFromVector3d(Eigen::Vector3d vector);

std::string getStringFromQuaternion(Eigen::Quaterniond q);

tf::Pose differenceBetweenPoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2);

double lengthOfVector(tf::Pose vector);

#endif