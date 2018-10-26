#include<math.h>
#include <FlexiTimer2.h>
#include <parameters.h>

#define MOTOR_TIME 0.2
#define SPEEDCONTROL_TIME 0.01

volatile int enc_val_right = 0, enc_val_left = 0;
volatile uint8_t enc_prev_right = 0, enc_prev_left = 0;
int PWM_Right, PWM_Left;
double glOmega = 0.00, glDeg = 0.00;
double Xcur=0.00, Ycur=0.00;
double distance_right = 0.00, distance_left = 0.00;

double kakunin1=35.00, kakunin2=30.00;
int flg = 0;

void space(){
  Serial.print("  ");
}

void setup() {
  parameters();

  attachInterrupt(5, updateEncoder1, CHANGE);
  attachInterrupt(4, updateEncoder1, CHANGE);
  attachInterrupt(3, updateEncoder2, CHANGE);
  attachInterrupt(2, updateEncoder2, CHANGE);

  FlexiTimer2::set(SPEEDCONTROL_TIME * 1000, odmetry);
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
    enc_val_right --;
  }
  else if(encoded == 0b1110 || encoded == 0b0111 || encoded == 0b0001 || encoded == 0b1000){
    enc_val_right ++;
  }

  enc_prev_right = ab;
  flg = 1;
}

void updateEncoder2(){
//  uint8_t c = digitalRead(20);
//  uint8_t d = digitalRead(21);
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
    enc_val_left --;
  }
  else if(encoded == 0b1110 || encoded == 0b0111 || encoded == 0b0001 || encoded == 0b1000){
    enc_val_left ++;
  }

  enc_prev_left = cd;
  flg = 1;
}

void odmetry(){
  double glVelocity = 0.00;
  double rad = 0.00;

  glOmega = enc_val_right - enc_val_left;     //for omega 誤差(?)出るからパルス=>radは後で
  glDeg += glOmega;
/*==============================================================================
from ball-zone to goal => clockwise
opposite               => CCW
  -> countermeasure of overflow
================================================================================*/

  glVelocity = (enc_val_right + enc_val_left) / 2 * PULSE_TO_MM;
  rad = glDeg * PULSE_TO_MM / TREAD;
  Xcur += glVelocity * sin(rad);         //x-cordinate => "sin"
  Ycur += glVelocity * cos(rad);         //y-cordinate => "cos"

  enc_val_right = 0;
  enc_val_left = 0;

}

void motorControl(int PWM_Right, int PWM_Left){
  int i;
  int duty_Right = PWM_MAX/PWM_Right, duty_Left = PWM_MAX/PWM_Left;
  for(i = 1; i <= PWM_MAX; i++){
    if((i%duty_Right) == 0){
      PORTG |= B00100000;
//      digitalWrite(4,HIGH);
    }
    else{
      PORTG &= ~B00100000;
//      digitalWrite(4,LOW);
    }

    if((i%duty_Left) == 0){
      PORTH |= B00001000;
//      digitalWrite(6, HIGH);
    }
    else{
      PORTH &= ~B00001000;
//      digitalWrite(6, LOW);
    }
  }
}

void motorStop(){
  PORTG &= ~B00100000;
  PORTH &= ~B00001000;
}

void goStraight(){
  double Xref = 0.00, Yref = 100.00;
  double deg;
  deg = glDeg * PULSE_TO_MM / TREAD * 180.00 / PI;
  deg *= 1.123;                               //miracle magic number
//    Serial.println(deg);


//  deff_deg = deg + acos((Yref-Ycur)/sqrt((Xref-Xcur)*(Xref-Xcur)+(Yref-Ycur)*(Yref-Ycur)))*180/PI;
  if(flg == 1){
//    kakunin1 = -deff_deg * pGainRight;
//    kakunin2 = +deff_deg * pGainLeft;

      PWM_Right = glOmega * PGAIN_R + (Yref-Ycur) * IGAIN_R;
      PWM_Left = glOmega * PGAIN_L + (Yref-Ycur) * IGAIN_L;
    if(PWM_Right > 256){
      PWM_Right = 256;
    }
    else if(PWM_Right <= 0){
      PWM_Right = 1;
    }
    if(PWM_Left > 256){
      PWM_Left = 256;
    }
    else if(PWM_Left <= 0){
      PWM_Left = 1;
    }

  }
  motorControl(PWM_Right, PWM_Left);
  flg = 0;
  if(Yref <= Ycur){
    Yref += 100;
  }
//Serial.println(kakunin2);
}
void lineTrace(){
  int Ana0, Ana1, Ana2, Ana3;
  Ana0 = analogRead(A0);
  Ana1 = analogRead(A1);
  Ana2 = analogRead(A2);
  Ana3 = analogRead(A3);

  if(Ana1 < 100){
    PWM_Right -= 40;
    PWM_Left += 20;
    motorControl(PWM_Right, PWM_Left);
  }
  else if(Ana2 < 100){
    PWM_Right += 20;
    PWM_Left -= 40;
    motorControl(PWM_Right, PWM_Left);
  }
  else{
    goStraight();
  }

Serial.print(Ana0);
space();
Serial.print(Ana1);
space();
Serial.print(Ana2);
space();
Serial.println(Ana3);
}

void loop(){
  lineTrace();
}
