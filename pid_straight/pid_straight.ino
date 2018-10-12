#include<stdio.h>
#include<math.h>
#include <FlexiTimer2.h>

#define TIME 0.20
#define RADIUS 50           
#define GEAR_RATIO 116160
#define VPGAIN_r 0.15
#define VDGAIN_r 0.0
#define VIGAIN_r 0.02
#define VPGAIN_l 0.15
#define VDGAIN_l 0.0
#define VIGAIN_l 0.02
#define MOVE_PER_PULSE 0.02136585468                      //1pulseで進む距離[mm]
#define SPEED_RIGHT_REF 200                               //target speed[mm/s]
#define SPEED_LEFT_REF 200

double motor_right, motor_left;
 
volatile int enc_val_right = 0, enc_val_left = 0;
volatile uint8_t enc_prev1 = 0, enc_prev2 = 0;


void setup(){
//encoder setting
  pinMode(18, INPUT);                                   //encoder r
  pinMode(19, INPUT);
  pinMode(20, INPUT);                                  //encoder l
  pinMode(21, INPUT);
  attachInterrupt(5, updateEncoder1, CHANGE);           //interrupt pin18=5, pin19=4
  attachInterrupt(4, updateEncoder1, CHANGE);
  attachInterrupt(3, updateEncoder2, CHANGE);
  attachInterrupt(2, updateEncoder2, CHANGE);

//motor setting
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);  
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(18, HIGH);
  digitalWrite(19, HIGH);

  FlexiTimer2::set(TIME*1000, motorControl); // 200ms period
  FlexiTimer2::start();
  
  Serial.begin(9600);
}

void updateEncoder1(){
  uint8_t a = digitalRead(18);
  uint8_t b = digitalRead(19);
 
  uint8_t ab = (a << 1) | b;
  uint8_t encoded  = (enc_prev1 << 2) | ab;

  if(encoded == 0b1101 || encoded == 0b0100 || encoded == 0b0010 || encoded == 0b1011){
    enc_val_right ++;
  } else if(encoded == 0b1110 || encoded == 0b0111 || encoded == 0b0001 || encoded == 0b1000) {
    enc_val_right --;
  }

  enc_prev1 = ab;
}

void updateEncoder2(){
  uint8_t c = digitalRead(20);
  uint8_t d = digitalRead(21);
 
  uint8_t cd = (c << 1) | d;
  uint8_t encoded  = (enc_prev2 << 2) | cd;

  if(encoded == 0b1101 || encoded == 0b0100 || encoded == 0b0010 || encoded == 0b1011){
    enc_val_left ++;
  } else if(encoded == 0b1110 || encoded == 0b0111 || encoded == 0b0001 || encoded == 0b1000) {
    enc_val_left --;
  }

  enc_prev2 = cd;
}

//gammma -> 限界値に近ずけるための定数
double low_pass_filter(double val, double pre_val, double gamma){
  return gamma * pre_val + (1-gamma) * val;
}
  

void motorControl(){
  double speed_right, speed_left;
  double speed_right_lpf =0.0, speed_left_lpf =0.0;         //Low Pass Filterの初期化 motor
  double difference_speed_right, difference_speed_left;
  double i_right = 0.0, i_left = 0.0;

  speed_right = MOVE_PER_PULSE * enc_val_right / TIME;
  speed_left = MOVE_PER_PULSE * enc_val_left / TIME;
  speed_right_lpf = low_pass_filter(speed_right, speed_right_lpf, 0.5);
  speed_left_lpf = low_pass_filter(speed_left, speed_left_lpf, 0.5);
  
  difference_speed_right = SPEED_RIGHT_REF - speed_right_lpf;
  difference_speed_left = SPEED_LEFT_REF - speed_left_lpf;
  
  i_right += difference_speed_right * TIME;
  i_left += difference_speed_left * TIME;
  
  motor_right = VPGAIN_r*difference_speed_right + VIGAIN_r * i_right;
  motor_left = VPGAIN_l*difference_speed_left + VIGAIN_l * i_left;

  enc_val_right=0;
  enc_val_left=0;
}


void loop(){
 analogWrite(4, motor_right);                                          
 analogWrite(5, 0);
 analogWrite(6, 0);
 analogWrite(7, motor_left);
 
//.確認用
  Serial.print(speed_right);
  Serial.print("  ");
  Serial.print(motor_right);
  Serial.println("  ");
}

 
