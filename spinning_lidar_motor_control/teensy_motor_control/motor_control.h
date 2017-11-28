
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>



/************************       Constants       ************************/
const byte PWM_RESOLUTION = 16;

// Pins for the quadrature encoder, IR sensor and motor PWM signal
const byte PIN_QUAD_ENC_A = 2;
const byte PIN_QUAD_ENC_B = 3;
const byte IR_SENSOR_PIN = 7;
const byte MOTOR_PWM_PIN = 9;

// Encoder offset estimation
const double ENCODER_COUNTS_PER_ROTATION = 229376.0;

// Velocity control PID
const double kp = 5.0;
const double ki = 2.0;
const double kd = 0.0;




/************************       Variables       ************************/
// Velocity control PID
double desired_vel = M_PI;
double prev_angle = 0.0;
int prev_time = 0;
int PWM_value = 48950; //49500;
double vel = 0.0;
double diff_time = 0.0;
double prev_err = 0.0;
double curr_err = 0.0;
double diff_err = 0.0;
double sum_err = 0.0;
double PID_value = 0.0;

// Encoder
int i = 0;
int j = 0;
int time_per_cycle = 0;
int prev_time_cycle = 0;
double encoder_offset = 0.0;
Encoder motor_encoder(PIN_QUAD_ENC_A, PIN_QUAD_ENC_B);

// Motor on/off
bool motor_stopped = true;



// Procedure to estimate the current velocity of the motor
void estimate_velocity() 
{
    int curr_time = micros();
//    double curr_angle = fmod(2*PI*(motor_encoder.read() - encoder_offset) / ENCODER_COUNTS_PER_ROTATION, 2*PI);
    double curr_angle = fmod(2.0*PI*(motor_encoder.read()) / ENCODER_COUNTS_PER_ROTATION, 2.0*PI);
    diff_time = (curr_time - prev_time) / 1000000.0;
    if (abs(curr_angle - prev_angle) < 1e-9) 
    {
        vel = 0.0;
    }
    else
    {
        if (curr_angle > prev_angle)
        {
            vel = (curr_angle - prev_angle)/diff_time;
        }
        else 
        {
            vel = (2*PI + curr_angle - prev_angle)/diff_time;
        }
    }

    prev_angle = curr_angle;
    prev_time = curr_time;
}



// PID Controller
void compute_PID()
{
    curr_err = desired_vel - vel;
    sum_err = sum_err + (curr_err * diff_time);
    diff_err = (curr_err - prev_err)/diff_time;
    prev_err = curr_err;
    PID_value = kp*curr_err + ki*sum_err + kd*diff_err;
}


// This procedure runs the PID controller and sends motion command to the motors
void control_motor()
{
    compute_PID();
    PWM_value = constrain(PWM_value + (int)PID_value, 48000, 65500);
    analogWrite(MOTOR_PWM_PIN, PWM_value);
}


// Just set the Motor PWM to the minimum to stop it
void stop_motor()
{
    estimate_velocity();
    analogWrite(MOTOR_PWM_PIN, 48000);
}


// This procedure configures the Teensy pins for controlling the motors and reading the encoders
void motor_setup()
{
    analogWriteResolution(PWM_RESOLUTION);  // max; forward PWM value: 48950-65500 (slow-fast)
}
