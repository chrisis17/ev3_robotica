#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#define ENCODER_FREQUENCY                 10   //hz
#define MOTOR_FREQUENCY                  300   //hz

#define PWM_M2A    5
#define PWM_M2B    6

#define ENC_IN_LEFT_A 2
#define ENC_IN_LEFT_B 4

// Parameters for a 1000 RPM motor
#define LEFT_KP   30.00
#define LEFT_KD    4.00
#define LEFT_KI    0.00

unsigned long  left_oldtime_encoder = 0;
unsigned long  left_newtime_encoder;

unsigned long  left_oldtime_motor = 0;
unsigned long  left_newtime_motor;

int motor_control = 0;

float left_input = 0.0;
float left_temp = 0.0;
float left_enc_w = 0.0;
float left_motor_ref = 0.0;
float left_motor_err = 0.0;
float left_delta_t_enc = 0.0;
float left_delta_t_mtr = 0.0;
float left_fil_enc_w = 0.0;
float left_derivative = 0.0;
float left_integrative = 0.0;
float last_left_fil_enc_w = 0.0;
float last_left_enc_w = 0.0;
float last_left_motor_err = 0.0;
volatile int left_wheel_tick_count = 0.0;
volatile int last_left_wheel_tick_count = 0.0;

unsigned long tTime[4];


/********* Publishers *********/

std_msgs::Float32 left_w_msg;
ros::Publisher encoder("w_left", &left_w_msg);

std_msgs::Float32 debug_msg;
ros::Publisher debug("debug_left", &debug_msg);

/********* Subscribers *********/

void motor_cb(const std_msgs::Float32& motor_msg);
ros::Subscriber<std_msgs::Float32> motor("ctrl_left", motor_cb);
