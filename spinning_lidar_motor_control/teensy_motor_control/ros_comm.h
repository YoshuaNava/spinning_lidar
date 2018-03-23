/***********************************************
***       Alan Khudur, Yoshua Nava - KTH     ***
************************************************/

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <spinning_lidar_motor_control/MotorState.h>
#include <spinning_lidar_motor_control/TurnMotorOnOff.h>
#include <spinning_lidar_motor_control/ChangeTargetVelocity.h>


using spinning_lidar_motor_control::MotorState;
using spinning_lidar_motor_control::TurnMotorOnOff;
using spinning_lidar_motor_control::ChangeTargetVelocity;




/************************       Constants       ************************/
const long BAUD_RATE = 115200;
const char* SENSOR_FRAME = "/laser_axis";
const char* SPINNING_JOINT = "lidar_spin_joint";




/************************       Variables       ************************/
// ROS node handler, publisher and services
ros::NodeHandle nh;
MotorState motor_state_msg;
std_msgs::Empty empty_msg;
sensor_msgs::JointState joint_state_msg;
ros::Publisher motor_state_pub("spinning_lidar/motor_state", &motor_state_msg);
ros::Publisher ir_interrupt_pub("spinning_lidar/ir_interrupt", &empty_msg);
ros::Publisher joint_states_pub("spinning_lidar/joint_states", &joint_state_msg);

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



void initializeSensorMsgs()
{
  motor_state_msg.header.frame_id = SENSOR_FRAME;
  joint_state_msg.header.frame_id = SENSOR_FRAME;
  joint_state_msg.name_length = 1;
  joint_state_msg.position_length = 1;
  joint_state_msg.velocity_length = 1;
  joint_state_msg.effort_length = 1;

  received_desired_vel = desired_vel;
}


void ros_setup()
{
  nh.getHardware()->setBaud(BAUD_RATE);
  nh.initNode();
  nh.advertise(motor_state_pub);
  nh.advertise(ir_interrupt_pub);
  nh.advertise(joint_states_pub);
  nh.advertiseService(motor_onoff_server);
  nh.advertiseService(change_vel_server);

  initializeSensorMsgs();


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


inline void publish_joint_states(double curr_angle, double curr_vel)
{
  joint_state_msg.header.stamp = nh.now();

  char* name[] = {(char*) SPINNING_JOINT};
  float pos[] = {(float) curr_angle};
  float vel[] = {(float) curr_vel};
  float eff[] = {0};
  joint_state_msg.name = name;
  joint_state_msg.position = pos;
  joint_state_msg.velocity = vel;
  joint_state_msg.effort = eff;

  joint_states_pub.publish(&joint_state_msg);
}
