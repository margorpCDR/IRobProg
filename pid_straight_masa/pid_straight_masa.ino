#include<math.h>
#include <FlexiTimer2.h>
#include <parameters.h>
#define MOTOR_TIME 0.2
#define ODOMETRY_TIME 0.01

volatile int enc_val_right = 0, enc_val_left = 0;
volatile uint8_t enc_prev_right = 0, enc_prev_left = 0;

double glDeg = 0.00;

double speed_right_lpf, speed_left_lpf;
double motor_right, motor_left;

void setup() {
  parameters();

  attachInterrupt(5, updateEncoder1, CHANGE);
  attachInterrupt(4, updateEncoder1, CHANGE);
  attachInterrupt(3, updateEncoder2, CHANGE);
  attachInterrupt(2, updateEncoder2, CHANGE);

  FlexiTimer2::set(MOTOR_TIME * 1000,motorControl);
  FlexiTimer2::start();
  
  Serial.begin(9600);
}


void updateEncoder1(){
//  uint8_t a = digitalRead(18);
//  uint8_t b = digitalRead(19);

  uint8_t a, b;
  if(PIND & _BV(PD3)){
    a = HIGH;
  }
  else{
    a = LOW;
  }
  if(PIND & _BV(PD2)){
    b = HIGH;
  }
  else{
    b = LOW;
  }

  uint8_t ab = (a << 1) | b;
  uint8_t encoded  = (enc_prev_right << 2) | ab;

  if(encoded == 0b1101 || encoded == 0b0100 || encoded == 0b0010 || encoded == 0b1011){
    enc_val_right ++;
  }
  else if(encoded == 0b1110 || encoded == 0b0111 || encoded == 0b0001 || encoded == 0b1000){
    enc_val_right --;
  }

  enc_prev_right = ab;
}

void updateEncoder2(){
//  uint8_t c = digitalRead(2);
//  uint8_t d = digitalRead(3);
  uint8_t c, d;
  if(PIND & _BV(PD1)){
    c = HIGH;
  }
  else{
    c = LOW;
  }
  if(PIND & _BV(PD0)){
    d = HIGH;
  }
  else{
    d = LOW;
  }

  uint8_t cd = (c << 1) | d;
  uint8_t encoded  = (enc_prev_left << 2) | cd;

  if(encoded == 0b1101 || encoded == 0b0100 || encoded == 0b0010 || encoded == 0b1011){
    enc_val_left ++;
  }
  else if(encoded == 0b1110 || encoded == 0b0111 || encoded == 0b0001 || encoded == 0b1000){
    enc_val_left --;
  }

  enc_prev_left = cd;
}

double low_pass_filter(double val, double pre_val, double gamma){                              //gammma 限界値に近ずけるための定数
 return gamma * pre_val + (1-gamma)*val;  
  }
  

void motorControl(){   
  int rRet, lRet, rRetBefore, lRetBefore;                                                                        
  double i_right, i_left;
  double speed_right = PULSE_TO_MM*enc_val_right /MOTOR_TIME;
  double speed_left = PULSE_TO_MM *enc_val_left / MOTOR_TIME;
  speed_right_lpf = low_pass_filter(speed_right, speed_right_lpf, 0.4);
  speed_left_lpf = low_pass_filter(speed_left, speed_left_lpf, 0.4);
  
  double difference_speed_right = SPEED_RIGHT_REF - speed_right_lpf;
  double difference_speed_left = SPEED_LEFT_REF - speed_left_lpf;
  
  i_right += difference_speed_right * MOTOR_TIME;
  i_left += difference_speed_left * MOTOR_TIME;
  
  if(i_right>255) i_right= 255;                                          //積分上限
  if(i_left>255) i_left = 255;

  motor_right =VPGAIN_r*difference_speed_right + VIGAIN_r*i_right;
  motor_left = VPGAIN_l*difference_speed_left + VIGAIN_l*i_left;
  
  enc_val_right=0;
  enc_val_left=0;

}     


void loop() {
  
 motorControl();
 analogWrite(6,0);                                          
 analogWrite(7,motor_right);
 analogWrite(8,motor_left);
 analogWrite(11,0);


  Serial.print(SPEED_RIGHT_REF);
  Serial.print(" ");
  Serial.print(SPEED_LEFT_REF);
  Serial.print(" ");
  Serial.print(speed_right_lpf);
  Serial.print(" ");
  Serial.println(speed_left_lpf);
 
                         //.確認用
  
}

 


