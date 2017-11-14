
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

/************************       Constants       ************************/
// Encoder offset estimation
const int ENCODER_OFFSET_ARRAY_SIZE = 20;

// Pins for the quadrature encoder, IR sensor and motor PWM signal
const byte PIN_QUAD_ENC_A = 2;
const byte PIN_QUAD_ENC_B = 3;
const byte IR_SENSOR_PIN = 7;
const byte MOTOR_PWM_PIN = 9;

// Velocity control PID
const double kp = 5.0;
const double ki = 2.0;
const double kd = 0.0;


/************************       Variables       ************************/
// Velocity control PID
double prev_angle = 0.0;
int prev_time = 0;
int prev_time_new_turn = 0;
int PWM_value = 48950; //49500;
int i = 0;
int j = 0;
int iteration = 0;
double velocity = 0.0;
double diff_time = 0.0;
double prev_err = 0.0;
double sum_err = 0.0;
int time_for_one_loop = 0;


// Encoder
double encoder_offset = 0.0;
double encoder_offset_sum = 0.0;
double encoder_offset_array[ENCODER_OFFSET_ARRAY_SIZE];
Encoder motor_encoder(PIN_QUAD_ENC_A, PIN_QUAD_ENC_B);


void new_turn()
{
    
}

void motor_setup()
{
    pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), new_turn, RISING);
    analogWriteResolution(16);  // max; forward PWM value: 48950-65500 (slow-fast)
}