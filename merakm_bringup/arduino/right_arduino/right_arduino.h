#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#define ENCODER_FREQUENCY                 10   //hz
#define MOTOR_FREQUENCY                  300   //hz

#define PWM_M2A    6
#define PWM_M2B    5

//#define PWM_M2A    8
//#define PWM_M2B    9

// Parameters for a 1000 RPM motor
#define RIGHT_KP   30.00
#define RIGHT_KD    4.00
#define RIGHT_KI    1.00

// Parameters for a 37 RPM motor
//#define RIGHT_KP  400.00
//#define RIGHT_KD   90.00
//#define RIGHT_KI    0.00

//#define ENC_IN_RIGHT_A 21
//#define ENC_IN_RIGHT_B 20

#define ENC_IN_RIGHT_A 3
#define ENC_IN_RIGHT_B 2

unsigned long  right_oldtime_encoder = 0;
unsigned long  right_newtime_encoder;

unsigned long  right_oldtime_motor = 0;
unsigned long  right_newtime_motor;

int motor_control = 0;

float right_input = 0.0;
float right_temp = 0.0;
float right_enc_w = 0.0;
float right_motor_ref = 0.0;
float right_motor_err = 0.0;
float right_delta_t_enc = 0.0;
float right_delta_t_mtr = 0.0;
float right_fil_enc_w = 0.0;
float right_derivative = 0.0;
float right_integrative = 0.0;
float last_right_fil_enc_w = 0.0;
float last_right_enc_w = 0.0;
float last_right_motor_err = 0.0;
volatile int right_wheel_tick_count = 0.0;
volatile int last_right_wheel_tick_count = 0.0;

unsigned long tTime[4];


/********* Publishers *********/

std_msgs::Float32 right_w_msg;
ros::Publisher encoder("encoder", &right_w_msg);

std_msgs::Float32 debug_msg;
ros::Publisher debug("debug", &debug_msg);

/********* Subscribers *********/

void motor_cb(const std_msgs::Float32& motor_msg);
ros::Subscriber<std_msgs::Float32> motor("motor", motor_cb);
