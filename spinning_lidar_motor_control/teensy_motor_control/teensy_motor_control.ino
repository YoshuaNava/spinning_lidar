
#include <math.h>
#include "motor_control.h"
#include "ros_comm.h"

/************************       Constants       ************************/
const int MAIN_LOOP_FREQ = 40;
const double MAIN_LOOP_DELAY = 1000000.0/MAIN_LOOP_FREQ;

const long ZERO = 0;

// This interrupt estimates the encoder offset, and estimates its current position and velocity
void interrupt_IR_sensor()
{
    encoder_lock = true;
    encoder_offset = motor_encoder.read();
    angle_offset = fmod(2.0*PI*(encoder_offset) / ENCODER_COUNTS_PER_ROTATION, 2.0*PI);
    motor_encoder.write(ZERO);
    encoder_lock = false;
    publish_ir_interrupt();
}


// Setup pin and callback for IR sensor interrupt
void IR_interrupt_setup()
{
    pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), interrupt_IR_sensor, RISING);
}


// Setup:
void setup() 
{
    ros_setup();
    motor_setup();
    IR_interrupt_setup();
}

// Loop:
void loop() 
{
    int time_now = micros();
    
    if(received_desired_vel != desired_vel)
    {
        desired_vel = received_desired_vel;
    }

    if(!encoder_lock)
    {
        estimate_velocity();

        if(motor_stopped)
        {
            stop_motor();
        }
        else
        {
            control_motor();
        }
    }

    publish_motor_state(motor_stopped, prev_angle, angle_offset, vel);
    
    nh.spinOnce();
    delay((MAIN_LOOP_DELAY - (micros() - time_now))/1000);
}

