
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <spinning_lidar_motor_control/MotorState.h>
#include <spinning_lidar_motor_control/TurnMotorOnOff.h>
#include <spinning_lidar_motor_control/ChangeTargetVelocity.h>


using spinning_lidar_motor_control::MotorState;
using spinning_lidar_motor_control::TurnMotorOnOff;
using spinning_lidar_motor_control::ChangeTargetVelocity;




/************************       Constants       ************************/
const long BAUD_RATE = 57600;
const char* SENSOR_FRAME = "/laser_axis";




/************************       Variables       ************************/
// ROS node handler, publisher and services
ros::NodeHandle nh;
MotorState motor_state_msg;
std_msgs::Empty empty_msg;
ros::Publisher motor_state_pub("spinning_lidar/motor_state", &motor_state_msg);
ros::Publisher ir_interrupt_pub("spinning_lidar/ir_interrupt", &empty_msg);

void motor_onoff_cb(const TurnMotorOnOff::Request &req, TurnMotorOnOff::Response &reply)
{
    motor_stopped = req.stopped;
    
    reply.success = true;
}
ros::ServiceServer<TurnMotorOnOff::Request, TurnMotorOnOff::Response> motor_onoff_server("spinning_lidar/turn_motor_onoff",&motor_onoff_cb);

void reset_encoder_cb(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &reply)
{
    motor_encoder.write(0);
}
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> reset_encoder_server("spinning_lidar/reset_encoder",&reset_encoder_cb);

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
    nh.advertise(ir_interrupt_pub);
    nh.advertiseService(motor_onoff_server);
    nh.advertiseService(change_vel_server);
    nh.advertiseService(reset_encoder_server);
    motor_state_msg.header.frame_id = SENSOR_FRAME;

    received_desired_vel = desired_vel;

    delay(20);
}


inline void publish_ir_interrupt()
{
    ir_interrupt_pub.publish(&empty_msg);
}


inline void publish_motor_state(bool stopped, double curr_angle, double offset_angle, double curr_vel)
{
    motor_state_msg.header.stamp = nh.now();
    motor_state_msg.stopped = stopped;
    motor_state_msg.curr_angle = curr_angle;
    motor_state_msg.offset_angle = offset_angle;
    motor_state_msg.curr_vel = curr_vel;
    motor_state_msg.des_vel = desired_vel;
    motor_state_pub.publish(&motor_state_msg);
}



