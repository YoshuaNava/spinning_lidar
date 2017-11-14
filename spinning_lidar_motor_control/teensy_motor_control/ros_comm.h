
#include <ros.h>
#include <spinning_lidar_motor_control/MotorState.h>

/************************       Constants       ************************/
const int BAUD_RATE = 57600;


/************************       Variables       ************************/
// ROS node handler, messages and publishers/subscribers
ros::NodeHandle nh;
spinning_lidar_motor_control::MotorState motor_state_msg;
ros::Publisher motor_state_pub("spinning_lidar/motor_state", &motor_state_msg);


void ros_setup()
{
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    nh.advertise(motor_state_pub);
    motor_state_msg.header.frame_id = "/odom";

    delay(20);
}


// Serial print messages:
inline void publish_motor_state(bool stopped, float angle, float velocity)
{
    motor_state_msg.header.stamp = nh.now();
    motor_state_msg.stopped = stopped;
    motor_state_msg.angle = angle; //PWM_value;
    motor_state_msg.rot_vel = velocity;
    motor_state_pub.publish(&motor_state_msg);
    nh.spinOnce();
}
