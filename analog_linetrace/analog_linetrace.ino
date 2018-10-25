#include<stdio.h>
#include<math.h>
#define GEAR_RATIO 9900

double move_per_pulse = 0.01586662956;
double VPGAIN_analog = 0.0;
double VIGAIN_analog = 0.0;
double VDGAIN_analog = 0.0;
double analog_value0_lpf =0.0, analog_value1_lpf =0.0, analog_value2_lpf =0.0,analog_value3_lpf =0.0, analog_value4_lpf =0.0;
volatile int value1 = 0, value2 = 0;
volatile uint8_t prev1 = 0, prev2 = 0;
long rEnc = 0.0;
long rEncoder= 0.0;
long lEnc = 0.0;
long lEncoder = 0.0;
int line= 0;

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

void analog_trace_Control(){                                                                     //analogtraceのmax1000としてプログラムしてるから訂正すべき

int digital_value0 = digitalRead(A0);
int analog_value1 = analogRead(A1);
int analog_value2 = analogRead(A2);
int analog_value3 = analogRead(A3);
int digital_value4 = digitalRead(A4);

double analog_motor_right;
double analog_motor_left;
map(analog_motor_right,0,255,0,1000);
map(analog_motor_left,0,255,0,1000);

//analog_value0_lpf = low_pass_filter(analog_value0, analog_value0_lpf, 0.5);
analog_value1_lpf = low_pass_filter(analog_value1, analog_value1_lpf, 0.5);
analog_value2_lpf = low_pass_filter(analog_value2, analog_value2_lpf, 0.5);
analog_value3_lpf = low_pass_filter(analog_value3, analog_value3_lpf, 0.5);
//analog_value4_lpf = low_pass_filter(analog_value4, analog_value2_lpf, 0.5);

if(analog_value1_lpf > 1000)  analog_value1_lpf = 1000;
if(analog_value2_lpf > 1000)  analog_value2_lpf = 1000;
if(analog_value3_lpf > 1000)  analog_value3_lpf = 1000;


int analog_value_diff = analog_value1 - analog_value3;


/*  以下考える必要あり　＝＞　value123の値に応じて左右にハンドル操作
 if(analog_value_diff == 0){
 analogWrite(6,analog_motor_right);
 analogWrite(11,analog_motor_left); 
 }
 if(analog_value2>0 && analog_value_diff>0){
   analogWrite(6,analog_motor_right-analog_value_diff);
   analogWrite(11,analog_motor_left);
 }
 if(analog_value2>0 && analog_value_diff<0){
   analogWrite(6,analog_motor_right);
   analogWrite(11,analog_motor_left-(analog_value_diff*-1));
 }
*/
}

void lineCount(){
 rEncoder = rEnc + value1; 
 if(value1>30000){
 rEnc += value1;
 value1=0;
 }else if(value1<-30000){
 rEnc += value1;
 value1=0;
  }
  
 lEncoder = lEnc + value2; 
 if(value2>30000){
 lEnc += value2;
 value2=0;
 }else if(value2<-30000){
 lEnc+= value2;
 value2=0;
  }

if(analog_value0_lpf<500 || analog_value4_lpf<500 || 000<value1<000 || 000<value2<000){
  line++;
  rEncoder=0, lEncoder=0; 
  }
}

void loop() {
 lineCount(); 
 analog_trace_Control();

 Serial.print(value1);
 Serial.print("   ");
 Serial.println(value2);
 
}
