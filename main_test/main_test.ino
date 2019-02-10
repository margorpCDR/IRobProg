#include<stdio.h>
#include<math.h>
#include <FlexiTimer2.h>
#include <parameters.h>

#define MOTOR_LEFT 6                                   //ピン指定d6,,d7
#define MOTOR_RIGHT 7

int PWM_Left;
int PWM_Right;

int lineCounter = 0;                                       //linecount
int MaxLineCount = 4;
short flg = 0, preflg = 0;

volatile int enc_val_left = 0, enc_val_right = 0;        //encoder
volatile uint8_t enc_prev_left = 0, enc_prev_right = 0;

void setup() {
  
parameters();

// No.5 pin-setting for motor PWM
  TCCR3A = B10101011;
  TCCR3B = B00011001;
// No.6,7,8
  TCCR4A = B10101011;
  TCCR4B = B00011001;


attachInterrupt(0, updateEncoder1, CHANGE);
attachInterrupt(1, updateEncoder1, CHANGE);
attachInterrupt(4, updateEncoder2, CHANGE);
attachInterrupt(5, updateEncoder2, CHANGE);

Serial.begin(9600);
  
}


/************************ エンコーダのアルゴリズム　*******************************/
void updateEncoder1() {
  //  uint8_t a = digitalRead(18);
  //  uint8_t b = digitalRead(19);

  uint8_t a, b;
  if (PIND & _BV(PD2)) {
    a = HIGH;
  }
  else {
    a = LOW;
  }
  if (PIND & _BV(PD3)) {
    b = HIGH;
  }
  else {
    b = LOW;
  }

  uint8_t ab = (a << 1) | b;
  uint8_t encoded  = (enc_prev_right << 2) | ab;

  if (encoded == 0b1101 || encoded == 0b0100 || encoded == 0b0010 || encoded == 0b1011) {
    enc_val_right ++;
  }
  else if (encoded == 0b1110 || encoded == 0b0111 || encoded == 0b0001 || encoded == 0b1000) {
    enc_val_right --;
  }

  enc_prev_right = ab;
}

void updateEncoder2() {
  //  uint8_t c = digitalRead(2);
  //  uint8_t d = digitalRead(3);
  
  uint8_t c, d;
  if (PINE & _BV(PE5)) {
    c = HIGH;
  }
  else {
    c = LOW;
  }
  if (PINE & _BV(PE4)) {
    d = HIGH;
  }
  else {
    d = LOW;
  }

  uint8_t cd = (c << 1) | d;
  uint8_t encoded  = (enc_prev_left << 2) | cd;

  if (encoded == 0b1101 || encoded == 0b0100 || encoded == 0b0010 || encoded == 0b1011) {
    enc_val_left ++;
  }
  else if (encoded == 0b1110 || encoded == 0b0111 || encoded == 0b0001 || encoded == 0b1000) {
    enc_val_left --;
  }

  enc_prev_left = cd;
}

/***************************   モータの制御    ************************************/
/*void motorR(int PWM){
  if(PWM >= 0){
    OCR3A = PWM * 256;  //0~65536 => 0~255(No.5pin)
    OCR4A = 0;          //0~65536 => 0~255(No.6pin)
  }
  else{
    OCR3A = 0;          //0~65536 => 0~255(No.5pin)
    OCR4A = -PWM * 256; //0~65536 => 0~255(No.6pin)
  }
}

void motorL(int PWM){
  if(PWM >= 0){
    OCR4B = PWM * 256;  //0~65536 => 0~255(No.7pin)
    OCR4C = 0;          //0~65536 => 0~255(No.8pin)
  }
  else{
    OCR4B = 0;          //0~65536 => 0~255(No.7pin)
    OCR4C = -PWM * 256; //0~65536 => 0~255(No.8pin)
  }
}

void motorControl() {
  int i;
  int duty_Right = PWM_MAX / PWM_Right, duty_Left = PWM_MAX / PWM_Left;
  for (i = 1; i <= PWM_MAX; i++) {
    if ((i % duty_Right) == 0) {
      PORTE |= B00001000;
      //      digitalWrite(5,HIGH);
    }
    else {
      PORTE &= ~B0001000;
      //      digitalWrite(5,LOW);
    }
    if ((i % duty_Left) == 0) {
      PORTH |= B00010000;
      //      digitalWrite(7, HIGH);
    }
    else {
      PORTH &= ~B00010000;
      //      digitalWrite(7, LOW);
    }
  }
}


void speedControl() {
  double deg;
  double speedRight = enc_val_right * PULSE_TO_MM / SPEEDCONTROL_TIME;
  double speedLeft = enc_val_left * PULSE_TO_MM / SPEEDCONTROL_TIME;
  double speedRight_lpf = 0.00, speedLeft_lpf = 0.00;
  speedRight_lpf = low_pass_filter(speedRight, speedRight_lpf, 0.4);
  speedLeft_lpf = low_pass_filter(speedLeft, speedLeft_lpf, 0.4);
  double diffSpeedRight = speedRefRight - speedRight_lpf;
  double diffSpeedLeft = speedRefLeft - speedLeft_lpf;
  deg = glDeg * PULSE_TO_MM / TREAD * 180.00 / PI;
//  Serial.println(deg);
//  deff_deg = deg + acos((Yref-Ycur)/sqrt((Xref-Xcur)*(Xref-Xcur)+(Yref-Ycur)*(Yref-Ycur)))*180/PI;
//  PWM_Right = diffSpeedRight * PGAIN_R;
//  PWM_Left = diffSpeedLeft * PGAIN_L;
  PWM_Right = diffSpeedRight * PGAIN_R + (diffSpeedRight * MOTOR_TIME) * IGAIN_R + glDeg * DGAIN_R;
  PWM_Left = diffSpeedLeft * PGAIN_L + (diffSpeedLeft * MOTOR_TIME) * IGAIN_L + glDeg * DGAIN_L;
  if (PWM_Right > 256) {
    PWM_Right = 256;
  }
  else if (PWM_Right <= 0) {
    PWM_Right = 1;
  }
  if (PWM_Left > 256) {
    PWM_Left = 256;
  }
  else if (PWM_Left <= 0) {
    PWM_Left = 1;
  }
  Serial.print(diffSpeedRight);
  Serial.print("  ");
  Serial.println(PWM_Right);
  
  motorControl();
  
}*/


void motorStop() {
  PORTG &= ~B00100000;
  PORTH &= ~B00001000;
}

/************************ライントレース ****************************************/

void lineCount(){
int analog_value1 = analogRead(A2);
int analog_value2 = analogRead(A3);
int analog_value3 = analogRead(A4);
int analog_value4 = analogRead(A5);
int analog_value5 = analogRead(A6);
if(analog_value2 > 1000)  analog_value2 = 1000;                                                  //上限を定める
if(analog_value3 > 1000)  analog_value3 = 1000;
if(analog_value4 > 1000)  analog_value4 = 1000;
if(analog_value1>801) analog_value1 = 1; else if(analog_value1<800) analog_value1 =0;            //デジタル値に似せる
if(analog_value5>801) analog_value5 = 1; else if(analog_value5<800) analog_value5 =0;

if(analog_value1 == 1 || analog_value5 == 1){
    flg = 1;
  }
  else{
    flg = 0;
  }

  if(flg - preflg == -1) lineCounter++;

  preflg = flg;
  
 /*Serial.print(analog_value1);  //left
 Serial.print("   ");
 Serial.print(analog_value2);
 Serial.print("   ");
 Serial.print(analog_value3);
 Serial.print("   ");
 Serial.print(analog_value4);
 Serial.print("   ");
 Serial.print(analog_value5);
 Serial.print("   ");
Serial.println(lineCounter);*/
}
//--------------------------------------------------------------------------------
//
//LINETRACE
//
//---------------------------------------------------------------------------------
void lineTrace(){

int analog_value1 = analogRead(A2);
int analog_value2 = analogRead(A3);
int analog_value3 = analogRead(A4);
int analog_value4 = analogRead(A5);
int analog_value5 = analogRead(A6);
if(analog_value2 > 1000)  analog_value2 = 1000;                                                  //上限を定める
if(analog_value3 > 1000)  analog_value3 = 1000;
if(analog_value4 > 1000)  analog_value4 = 1000;
if(analog_value1>801) analog_value1 = 1; else if(analog_value1<800) analog_value1 =0;            //デジタル値に似せる
if(analog_value5>801) analog_value5 = 1; else if(analog_value5<800) analog_value5 =0;
 
double analog_val_diff = analog_value2 - analog_value4;
 
if(analog_value2 <= 50 && analog_value4 <= 50 && analog_value3> 200){               //ここをいじらないとダメ
  PWM_Left = SPEED_LEFT_REF;
  PWM_Right = SPEED_RIGHT_REF;
 } 
 if(analog_val_diff > 0){             //右ずれ　
  PWM_Left = SPEED_LEFT_REF - analog_val_diff * VPGAIN_l;
  PWM_Right = SPEED_RIGHT_REF;
 }
 if(analog_val_diff<0){               //左ずれ
  PWM_Left = SPEED_LEFT_REF;
  PWM_Right = SPEED_RIGHT_REF + analog_val_diff * VPGAIN_r;
 }

//ここまで

 Serial.print(analog_value1);  //left
 Serial.print("   ");
 Serial.print(analog_value2);
 Serial.print("   ");
 Serial.print(analog_value3);
 Serial.print("   ");
 Serial.print(analog_value4);
 Serial.print("   ");
 Serial.print(analog_value5); //rigth
 Serial.print("   ");
 Serial.print(analog_val_diff);
 Serial.print("   ");
 Serial.print("motor");
 Serial.print(PWM_Left);
 Serial.print("   ");
 Serial.println(PWM_Right);


};

void loop() {
 lineTrace(); 
 lineCount();
 analogWrite(MOTOR_LEFT,PWM_Left);
 analogWrite(MOTOR_RIGHT,PWM_Right);

 if(lineCounter == 1){
  motorStop();
  }
  
}
