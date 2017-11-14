
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
const int LOOPS_FOR_INIT = 10;
const int LOOPS_FOR_ESTIMATION = 50;
const int LOOPS_FOR_DRIFT_REESTIMATION = 1000;
const int ENCODER_OFFSET_ARRAY_SIZE = 20;
const double ENCODER_COUNTS_PER_ROTATION = 229376.0;

// Velocity control PID
const double kp = 5.0;
const double ki = 2.0;
const double kd = 0.0;


/************************       Variables       ************************/
// Velocity control PID
double desired_velocity = M_PI;
double prev_angle = 0.0;
int prev_time = 0;
int PWM_value = 48950; //49500;
double velocity = 0.0;
double diff_time = 0.0;
double prev_err = 0.0;
double current_err = 0.0;
double diff_err = 0.0;
double sum_err = 0.0;
double PID_value = 0.0;


// Encoder
int i = 0;
int j = 0;
int time_per_cycle = 0;
int prev_time_cycle = 0;
double encoder_offset = 0.0;
double encoder_offset_sum = 0.0;
double encoder_offset_array[ENCODER_OFFSET_ARRAY_SIZE];
Encoder motor_encoder(PIN_QUAD_ENC_A, PIN_QUAD_ENC_B);





void interrupt_IR_sensor()
{
    int time_now = micros();
    if (i < LOOPS_FOR_INIT) 
    {
        if (i == 0) 
        {
            prev_time_cycle = micros();
        }
        else
        {
            if (i == 1) 
            {
                time_per_cycle = time_now - prev_time_cycle;
            }
            else 
            {
                time_per_cycle = (time_per_cycle + (time_now - prev_time_cycle))/2;
            }
            prev_time_cycle = time_now;
        }
        i++;
    }
    else 
    {
        // Issue lies here:
        if (j < LOOPS_FOR_ESTIMATION) 
        {
            if ((time_now - prev_time_cycle) > time_per_cycle/2) 
            {
                encoder_offset_array[j] = fmod(motor_encoder.read(), ENCODER_COUNTS_PER_ROTATION);
                j++;

                double sum_of_array = 0.0;
                for (int k = 0; k < j; k++) 
                {
                    sum_of_array += encoder_offset_array[k];
                }
                encoder_offset = double(sum_of_array) / double(j);

                if (j == ENCODER_OFFSET_ARRAY_SIZE) 
                {
                    j = 0;  // restart
                }
            }
            prev_time_cycle = time_now;
        }
        else 
        {
            // do nothing
        }
    }
}


// Procedure to estimate the current velocity of the motor
void estimate_velocity() 
{
    int current_time = micros();
    double current_angle = fmod(2*PI*(motor_encoder.read() - encoder_offset) / ENCODER_COUNTS_PER_ROTATION, 2*PI);
    diff_time = (current_time - prev_time)/1000000.0;
    if (abs(current_angle - prev_angle) < 1e-9) 
    {
        velocity = 0.0;
    }
    else
    {
        if (current_angle > prev_angle)
        {
            velocity = (current_angle - prev_angle)/diff_time;
        }
        else 
        {
            velocity = (2*PI + current_angle - prev_angle)/diff_time;
        }
    }

    prev_angle = current_angle;
    prev_time = current_time;
}

// PID Controller
void compute_PID()
{
    current_err = desired_velocity - velocity;
    sum_err = sum_err + (current_err * diff_time);
    diff_err = (current_err - prev_err)/diff_time;
    prev_err = current_err;
    PID_value = kp*current_err + ki*sum_err + kd*diff_err;
}


void new_turn()
{
    
}

void control_motor()
{
    estimate_velocity();
    compute_PID();
    PWM_value = constrain(PWM_value + (int)PID_value, 48000, 65500);
    analogWrite(MOTOR_PWM_PIN, PWM_value);
}

void motor_setup()
{
    pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), interrupt_IR_sensor, RISING);
    analogWriteResolution(PWM_RESOLUTION);  // max; forward PWM value: 48950-65500 (slow-fast)
}
