
#include <math.h>
#include "motor_control.h"
#include "ros_comm.h"


bool motor_stopped = true;

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
    publish_motor_state(motor_stopped, 0, 0);
    delay((MAIN_LOOP_DELAY - (micros() - time_now))/1000);
}

