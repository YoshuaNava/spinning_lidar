#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "spinning_lidar_motor_control/MotorState.h"


std::string motor_state_topic;
std::string lidar_platform_link;
std::string lidar_axis_link;
std::string spinning_lidar_joint;
double startup_delay;

ros::Publisher spinning_lidar_joint_pub;


void motorStateCallback(const spinning_lidar_motor_control::MotorState::ConstPtr& msg)
{
    double angle = -msg->curr_angle;
    double vel = -msg->curr_vel;

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.header.frame_id = lidar_platform_link;
    joint_state_msg.name.push_back(spinning_lidar_joint);
    joint_state_msg.position.push_back(angle);
    joint_state_msg.velocity.push_back(vel);

    spinning_lidar_joint_pub.publish(joint_state_msg);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_jointstates_publisher_node");
	ros::NodeHandle nh;

    nh.param("motor_state_topic", motor_state_topic, std::string("/spinning_lidar/motor_state"));
    nh.param("lidar_platform_link", lidar_platform_link, std::string("laser_mount"));
    nh.param("lidar_axis_link", lidar_axis_link, std::string("laser_axis"));
    nh.param("spinning_lidar_joint", spinning_lidar_joint, std::string("lidar_spin_joint"));
    nh.param("startup_delay", startup_delay, 3.0);

    spinning_lidar_joint_pub = nh.advertise<sensor_msgs::JointState>("spinning_lidar/joint_states", 1);
    ros::Subscriber sub = nh.subscribe(motor_state_topic, 10, &motorStateCallback);

    ros::Duration(startup_delay).sleep();
    ROS_INFO("Publishing joint state of the rotating platform");
    ros::spin();

    return EXIT_SUCCESS;
}
    