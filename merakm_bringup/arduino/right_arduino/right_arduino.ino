#include "right_arduino.h"

ros::NodeHandle nh;

void setup() {

  nh.initNode();
  
  nh.advertise(encoder);
  nh.advertise(debug);
  nh.subscribe(motor);

  pinMode(ENC_IN_RIGHT_A , INPUT);
  pinMode(ENC_IN_RIGHT_B , INPUT);

  pinMode(PWM_M2A, OUTPUT);
  pinMode(PWM_M2B, OUTPUT);

  digitalWrite(PWM_M2A, LOW);       // MOTOR
  digitalWrite(PWM_M2B, LOW);

  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
}

void loop() {

  unsigned long t = micros();

  if ((t - tTime[0]) >= (1.0e6 / ENCODER_FREQUENCY))
  {
    right_newtime_encoder = micros();

    right_delta_t_enc = ((float) (right_newtime_encoder - right_oldtime_encoder))/1.0e6;
    right_temp = (right_wheel_tick_count - last_right_wheel_tick_count)/right_delta_t_enc;
    right_enc_w = right_temp*(2.0*PI)/(10.5*10);
    //right_enc_w = right_temp*(2.0*PI)/(11*10);

    right_fil_enc_w = 0.854*right_fil_enc_w + 0.0728*right_enc_w + 0.0728*last_right_fil_enc_w;
    
    right_w_msg.data = right_fil_enc_w;  
    encoder.publish(&right_w_msg);

    // Resolucion de 1.496 rad/s a una frecuencia de 100 Hz
    // Resolucion de 0.1496 rad/s a una frecuencia de 10 Hz

    last_right_fil_enc_w = right_fil_enc_w;
    last_right_wheel_tick_count = right_wheel_tick_count;
    right_oldtime_encoder = right_newtime_encoder;
    tTime[0] = t;
  }

  if ((t - tTime[1]) >= (1.0e6 / MOTOR_FREQUENCY))
  {
    right_newtime_motor = micros();
    
    right_motor_err = right_motor_ref - right_fil_enc_w;
    right_delta_t_mtr = ((float) (right_newtime_motor - right_oldtime_motor))/1.0e6;
    right_derivative = (right_motor_err - last_right_motor_err)/(right_delta_t_mtr);
    right_integrative = right_integrative + right_motor_err*right_delta_t_mtr;
    right_input = RIGHT_KP*right_motor_err + RIGHT_KD*right_derivative + RIGHT_KI*right_integrative;
    
    motor_control = right_input;
    motor_control = 0;

    debug_msg.data = right_input;
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

    last_right_motor_err = right_motor_err;
    right_oldtime_motor = right_newtime_motor;
    tTime[1] = t;
  }

  nh.spinOnce();
}


// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    right_wheel_tick_count--; 
  }
  else {
    right_wheel_tick_count++;
  }
}


void motor_cb(const std_msgs::Float32& motor_msg) {

  right_motor_ref = motor_msg.data;
}
