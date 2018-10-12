#include<stdio.h>
#include<math.h>
#include <FlexiTimer2.h>
#define TIME 0.20
#define RADIUS 50           
#define GEAR_RATIO 9900
#define VPGAIN_r 0.15
#define VDGAIN_r 0.0
#define VIGAIN_r 0.02
#define VPGAIN_l 0.15
#define VDGAIN_l 0.0
#define VIGAIN_l 0.02
double move_per_pulse = 0.01586662956;                   //1pulseで進む距離mm 重要

double i_right = 0.0;
double i_left = 0.0;

double motor_right;
double motor_left;

double speed_left_ref = 200, speed_right_ref= 200;         //目標速度
double speed_right_lpf =0.0, speed_left_lpf =0.0;        //low pass filterの初期化 motor

 
volatile int value1 = 0, value2 = 0;
volatile uint8_t prev1 = 0, prev2 = 0;


void setup() {
  
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(11,OUTPUT);
  
  pinMode(2, INPUT);        //encoder r
  pinMode(3, INPUT);
  pinMode(18,INPUT);        //encoder l
  pinMode(19,INPUT);
  
  attachInterrupt(0, updateEncoder1, CHANGE);
  attachInterrupt(1, updateEncoder1, CHANGE);
  attachInterrupt(4, updateEncoder2, CHANGE);
  attachInterrupt(5, updateEncoder2, CHANGE);
  
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(18,HIGH);
  digitalWrite(19,HIGH);

  FlexiTimer2::set(TIME*1000, motorControl); // 500ms period
  FlexiTimer2::start();
  
  Serial.begin(9600);
}

void updateEncoder1()
{
  uint8_t a = digitalRead(2);
  uint8_t b = digitalRead(3);
 
  uint8_t ab = (a << 1) | b;
  uint8_t encoded  = (prev1 << 2) | ab;

  if(encoded == 0b1101 || encoded == 0b0100 || encoded == 0b0010 || encoded == 0b1011){
    value1 ++;
  } else if(encoded == 0b1110 || encoded == 0b0111 || encoded == 0b0001 || encoded == 0b1000) {
    value1 --;
  }

  prev1 = ab;
}

void updateEncoder2()
{
  uint8_t c = digitalRead(18);
  uint8_t d = digitalRead(19);
 
  uint8_t cd = (c << 1) | d;
  uint8_t encoded  = (prev2 << 2) | cd;

  if(encoded == 0b1101 || encoded == 0b0100 || encoded == 0b0010 || encoded == 0b1011){
    value2 ++;
  } else if(encoded == 0b1110 || encoded == 0b0111 || encoded == 0b0001 || encoded == 0b1000) {
    value2 --;
  }

  prev2 = cd;
}

double low_pass_filter(double val, double pre_val, double gamma){                              //gammma 限界値に近ずけるための定数
 return gamma * pre_val + (1-gamma)*val;  
  }
  

void motorControl(){                                                                           
  
  double speed_right = move_per_pulse*value1 /TIME;
  double speed_left = move_per_pulse *value2 / TIME;
  speed_right_lpf = low_pass_filter(speed_right, speed_right_lpf, 0.5);
  speed_left_lpf = low_pass_filter(speed_left, speed_left_lpf, 0.5);
  
  double difference_speed_right = speed_right_ref - speed_right_lpf;
  double difference_speed_left = speed_left_ref - speed_left_lpf;
  
  i_right += difference_speed_right * TIME;
  i_left += difference_speed_left * TIME;
  
  motor_right = VPGAIN_r*difference_speed_right + VIGAIN_r*i_right;
  motor_left = VPGAIN_l*difference_speed_left + VIGAIN_l*i_left;

  value1=0;
  value2=0;

}     


void loop() {

 analogWrite(6,motor_right);                                          
 analogWrite(7,0);
 analogWrite(8,0);
 analogWrite(11,motor_left);
// delay(100);
 
                         //.確認用

  Serial.print(speed_right_lpf);
  Serial.print("  ");
  Serial.print(motor_right);
  Serial.println("  ");
 
}

 


