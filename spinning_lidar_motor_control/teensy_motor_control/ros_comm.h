
#include <ros.h>
#include <spinning_lidar_motor_control/MotorState.h>

/************************       Constants       ************************/
const int BAUD_RATE = 57600;
const char* SENSOR_FRAME = "/odom";


/************************       Variables       ************************/
// ROS node handler, messages and publishers/subscribers
ros::NodeHandle nh;
spinning_lidar_motor_control::MotorState motor_state_msg;
ros::Publisher motor_state_pub("spinning_lidar/motor_state", &motor_state_msg);
bool motor_stopped = true;


void ros_setup()
{
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    nh.advertise(motor_state_pub);
    motor_state_msg.header.frame_id = SENSOR_FRAME;

    delay(20);
}


// Serial print messages:
inline void publish_motor_state(bool stopped, double angle, double velocity)
{
    motor_state_msg.header.stamp = nh.now();
    motor_state_msg.stopped = stopped;
    motor_state_msg.angle = angle; //PWM_value;
    motor_state_msg.rot_vel = velocity;
    motor_state_pub.publish(&motor_state_msg);
    nh.spinOnce();
}
