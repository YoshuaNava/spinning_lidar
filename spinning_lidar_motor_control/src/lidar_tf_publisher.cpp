#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "spinning_lidar_motor_control/MotorState.h"


std::string motor_state_topic;
std::string lidar_platform_link;
std::string lidar_link;
double startup_delay;


void motorStateCallback(const spinning_lidar_motor_control::MotorState::ConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(msg->angle, 0.0, 0.0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), lidar_platform_link, lidar_link));
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_tf_publisher_node");
	ros::NodeHandle nh("~");

    nh.param("motor_state_topic", motor_state_topic, std::string("/spinning_lidar/motor_state"));
    nh.param("lidar_platform_link", lidar_platform_link, std::string("laser_platform"));
    nh.param("lidar_link", lidar_link, std::string("laser"));
    nh.param("startup_delay", startup_delay, 3.0);
    ros::Subscriber sub = nh.subscribe(motor_state_topic, 10, &motorStateCallback);

    ros::Duration(startup_delay).sleep();
    ROS_INFO("Broadcasting TF from the rotating platform to the lidar frame");
    ros::spin();

    return EXIT_SUCCESS;
}
    