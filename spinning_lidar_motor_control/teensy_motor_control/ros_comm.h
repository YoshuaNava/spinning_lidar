
#include <ros.h>
#include <spinning_lidar_motor_control/MotorState.h>
#include <spinning_lidar_motor_control/TurnMotorOnOff.h>
#include <spinning_lidar_motor_control/ChangeTargetVelocity.h>

using spinning_lidar_motor_control::MotorState;
using spinning_lidar_motor_control::TurnMotorOnOff;
using spinning_lidar_motor_control::ChangeTargetVelocity;




/************************       Constants       ************************/
const int BAUD_RATE = 57600;
const char* SENSOR_FRAME = "/laser";




/************************       Variables       ************************/
// ROS node handler, publisher and services
ros::NodeHandle nh;
MotorState motor_state_msg;
ros::Publisher motor_state_pub("spinning_lidar/motor_state", &motor_state_msg);

void motor_onoff_cb(const TurnMotorOnOff::Request &req, TurnMotorOnOff::Response &reply)
{
    motor_stopped = req.stopped;
    
    reply.success = true;
}
ros::ServiceServer<TurnMotorOnOff::Request, TurnMotorOnOff::Response> motor_onoff_server("spinning_lidar/turn_motor_onoff",&motor_onoff_cb);

double received_desired_vel = 0;
void vel_change_cb(const ChangeTargetVelocity::Request &req, ChangeTargetVelocity::Response &reply)
{
    motor_stopped = req.stopped;
    received_desired_vel = req.rot_vel;
    reply.success = true;
}
ros::ServiceServer<ChangeTargetVelocity::Request, ChangeTargetVelocity::Response> change_vel_server("spinning_lidar/change_motor_vel",&vel_change_cb);





void ros_setup()
{
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    nh.advertise(motor_state_pub);
    nh.advertiseService(motor_onoff_server);
    nh.advertiseService(change_vel_server);
    motor_state_msg.header.frame_id = SENSOR_FRAME;

    received_desired_vel = desired_vel;

    delay(20);
}



inline void publish_motor_state(bool stopped, double angle, double vel)
{
    motor_state_msg.header.stamp = nh.now();
    motor_state_msg.stopped = stopped;
    motor_state_msg.angle = angle; //PWM_value;
    motor_state_msg.curr_vel = vel;
    motor_state_msg.des_vel = desired_vel;
    motor_state_pub.publish(&motor_state_msg);
}
