
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

	void compose(Pose6DOF &p2)
	{
		this->pos = this->pos + this->rot.toRotationMatrix() * p2.pos;
		this->rot = this->rot * p2.rot;
		this->rot.normalize();
	}

	void substract(Pose6DOF &p2)
	{
		this->pos = this->pos - this->rot.toRotationMatrix() * p2.pos;
		this->rot = this->rot * p2.rot.inverse();
		this->rot.normalize();
	}

	static Pose6DOF compose(Pose6DOF &p1, Pose6DOF &p2)
	{
		Pose6DOF p3;
		p3.pos = p1.pos + p1.rot.toRotationMatrix() * p2.pos;
		p3.rot = p1.rot * p2.rot;
		p3.rot.normalize();
		return p3;
	}

	static Pose6DOF substract(Pose6DOF &p1, Pose6DOF &p2)
	{
		Pose6DOF p3;
		p3.pos = p1.pos - p1.rot.toRotationMatrix() * p2.pos;
		p3.rot = p1.rot * p2.rot.inverse();
		p3.rot.normalize();
		return p3;
	}

	void fromTMatrix(Eigen::Matrix4d &T)
	{
		this->pos = Eigen::Vector3d(-T(0,3), -T(1,3), -T(2,3));
		Eigen::Matrix3d rot = T.block(0, 0, 3, 3).transpose().cast<double>();
		this->rot = Eigen::Quaterniond(rot);
	}

	void fromROSPose(geometry_msgs::Pose &pose)
	{
		this->pos = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
		this->rot = Eigen::Quaterniond(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	}

	void fromTFPose(tf::Pose &pose)
	{
		this->pos = Eigen::Vector3d(pose.getOrigin().getX(), pose.getOrigin().getY(), pose.getOrigin().getZ());
		this->rot = Eigen::Quaterniond(pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());
	}
	
	void fromTFTransform(tf::Transform &transform)
	{
		this->pos = Eigen::Vector3d(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
		this->rot = Eigen::Quaterniond(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
	}

	tf::Transform toTFTransform()
	{
		tf::Pose transform;
		transform.setOrigin(tf::Vector3(this->pos(0), this->pos(1), this->pos(2)));
		transform.setRotation(tf::Quaternion(this->rot.x(), this->rot.y(), this->rot.z(), this->rot.w()));
		return transform;
	}

	tf::Pose toTFPose()
	{
		tf::Pose pose;
		pose.setOrigin(tf::Vector3(this->pos(0), this->pos(1), this->pos(2)));
		pose.setRotation(tf::Quaternion(this->rot.x(), this->rot.y(), this->rot.z(), this->rot.w()));
		return pose;
	}

private:
	const double EQUALITY_THRESH = 1e-10;
};


geometry_msgs::Point getROSPointFromPose6DOF(Pose6DOF pose);

Eigen::Matrix<double, 6, 6> getCovarianceFromROSPoseWithCovariance(geometry_msgs::PoseWithCovariance pose_msg);

tf::Transform getTFTransformFromROSOdometry(nav_msgs::Odometry odom_msg);

tf::Pose getTFPoseFromROSPose(geometry_msgs::Pose pose);

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