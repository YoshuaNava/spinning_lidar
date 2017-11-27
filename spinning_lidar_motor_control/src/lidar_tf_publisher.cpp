#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include "spinning_lidar_motor_control/MotorState.h"


std::string motor_state_topic;
std::string lidar_platform_link;
std::string lidar_axis_link;
double startup_delay;

ros::Publisher spinning_lidar_joint_pub;


void motorStateCallback(const spinning_lidar_motor_control::MotorState::ConstPtr& msg)
{
    double angle = -msg->angle;
    double vel = -msg->curr_vel;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(angle, 0.0, 0.0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), lidar_platform_link, lidar_axis_link));

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.position.push_back(angle);
    joint_state_msg.velocity.push_back(vel);

    spinning_lidar_joint_pub.publish(joint_state_msg);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_tf_publisher_node");
	ros::NodeHandle nh;

    nh.param("motor_state_topic", motor_state_topic, std::string("/spinning_lidar/motor_state"));
    nh.param("lidar_platform_link", lidar_platform_link, std::string("laser_mount"));
    nh.param("lidar_axis_link", lidar_axis_link, std::string("laser_axis"));
    nh.param("startup_delay", startup_delay, 3.0);

    spinning_lidar_joint_pub = nh.advertise<sensor_msgs::JointState>("spinning_lidar/joint_states", 1);
    ros::Subscriber sub = nh.subscribe(motor_state_topic, 10, &motorStateCallback);

    ros::Duration(startup_delay).sleep();
    ROS_INFO("Publishing joint state of the rotating platform");
    ROS_INFO("Broadcasting TF from the rotating platform to the lidar frame");
    ros::spin();

    return EXIT_SUCCESS;
}
    