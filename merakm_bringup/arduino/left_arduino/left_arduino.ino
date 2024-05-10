#include "left_arduino.h"

ros::NodeHandle nh;

void setup() {

  nh.initNode();
  
  nh.advertise(encoder);
  nh.advertise(debug);
  nh.subscribe(motor);

  pinMode(ENC_IN_LEFT_A , INPUT);
  pinMode(ENC_IN_LEFT_B , INPUT);

  pinMode(PWM_M2A, OUTPUT);
  pinMode(PWM_M2B, OUTPUT);

  digitalWrite(PWM_M2A, LOW);       // MOTOR
  digitalWrite(PWM_M2B, LOW);

  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
}

void loop() {

  unsigned long t = micros();

  if ((t - tTime[0]) >= (1.0e6 / ENCODER_FREQUENCY))
  {
    left_newtime_encoder = micros();

    left_delta_t_enc = ((float) (left_newtime_encoder - left_oldtime_encoder))/1.0e6;
    left_temp = (left_wheel_tick_count - last_left_wheel_tick_count)/left_delta_t_enc;
    left_enc_w = left_temp*(2.0*PI)/(10.5*10);

    left_fil_enc_w = 0.854*left_fil_enc_w + 0.0728*left_enc_w + 0.0728*last_left_fil_enc_w;
    
    left_w_msg.data = left_fil_enc_w*(-1.0);  
    encoder.publish(&left_w_msg);

    // Resolucion de 1.496 rad/s a una frecuencia de 100 Hz
    // Resolucion de 0.1496 rad/s a una frecuencia de 10 Hz

    last_left_fil_enc_w = left_fil_enc_w;
    last_left_wheel_tick_count = left_wheel_tick_count;
    left_oldtime_encoder = left_newtime_encoder;
    tTime[0] = t;
  }

  if ((t - tTime[1]) >= (1.0e6 / MOTOR_FREQUENCY))
  {
    left_newtime_motor = micros();
    
    left_motor_err = left_motor_ref - left_fil_enc_w*(-1.0);
    left_delta_t_mtr = ((float) (left_newtime_motor - left_oldtime_motor))/1.0e6;
    left_derivative = (left_motor_err - last_left_motor_err)/(left_delta_t_mtr);
    left_integrative = left_integrative + left_motor_err*left_delta_t_mtr;
    left_input = LEFT_KP*left_motor_err + LEFT_KD*left_derivative + LEFT_KI*left_integrative;
    
    motor_control = left_input;

    debug_msg.data = left_input;
    debug.publish(&debug_msg);

    if (motor_control == 0) {
      digitalWrite(PWM_M2B, LOW);
      digitalWrite(PWM_M2A, LOW);
    }
    else if (motor_control > 0 and motor_control <= 255) {
      analogWrite(PWM_M2B, motor_control);
      digitalWrite(PWM_M2A, LOW);
    }
    else if (motor_control > 255) {
      analogWrite(PWM_M2B, 255);
      digitalWrite(PWM_M2A, LOW);
    }
    else if (motor_control < 0 and motor_control >= -255) {
      digitalWrite(PWM_M2B, LOW);
      analogWrite(PWM_M2A, motor_control*(-1));
    }
    else if (motor_control < -255) {
      digitalWrite(PWM_M2B, LOW);
      analogWrite(PWM_M2A, 255);
    }

    last_left_motor_err = left_motor_err;
    left_oldtime_motor = left_newtime_motor;
    tTime[1] = t;
  }

  nh.spinOnce();
}


void left_wheel_tick() {
   
  int val = digitalRead(ENC_IN_LEFT_B);
  if(val == LOW) {
    left_wheel_tick_count--; 
  }
  else {
    left_wheel_tick_count++;
  }
}


void motor_cb(const std_msgs::Float32& motor_msg) {

  left_motor_ref = motor_msg.data;
}
