
#include <math.h>
#include "motor_control.h"
#include "ros_comm.h"


const int MAIN_LOOP_FREQ = 40;
const double MAIN_LOOP_DELAY = 1000000.0/MAIN_LOOP_FREQ;

// Setup:
void setup() 
{
    ros_setup();
    motor_setup();
}

// Loop:
void loop() 
{
    int time_now = micros();
    estimate_velocity();
    if(motor_stopped)
    {
        stop_motor();
    }
    else
    {
        control_motor();
    }

    if(received_desired_vel != desired_vel)
    {
        desired_vel = received_desired_vel;
    }
    publish_motor_state(motor_stopped, prev_angle, vel);
    nh.spinOnce();
    delay((MAIN_LOOP_DELAY - (micros() - time_now))/1000);
}

