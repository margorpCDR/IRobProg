#include<math.h>
#include <FlexiTimer2.h>
#include <parameters.h>

#define MOTOR_TIME 0.2
#define ODOMETRY_TIME 0.01

volatile int enc_val_right = 0, enc_val_left = 0;
volatile uint8_t enc_prev_right = 0, enc_prev_left = 0;
double glDeg = 0.00;
double  Xcur=0.00, Ycur=0.00;
double distance_right = 0.00, distance_left = 0.00;

double kakunin1=0.00, kakunin2=0.00;

void space(){
  Serial.print("  ");
}

void setup() {
  parameters();

  attachInterrupt(5, updateEncoder1, CHANGE);
  attachInterrupt(4, updateEncoder1, CHANGE);
  attachInterrupt(3, updateEncoder2, CHANGE);
  attachInterrupt(2, updateEncoder2, CHANGE);

  FlexiTimer2::set(ODOMETRY_TIME * 1000, odometry);
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
    enc_val_left ++;
  }
  else if(encoded == 0b1110 || encoded == 0b0111 || encoded == 0b0001 || encoded == 0b1000){
    enc_val_left --;
  }

  enc_prev_left = cd;
}

void odometry(){
  double Xref = 0.00, Yref = 0.00;
  double glOmega = 0.00, glVelocity = 0.00, glRho = 0.00, glTheta = 0.00;
  double rad = 0.00, deg = 0.00, deff_deg = 0.00, velocity = 0.00;
  double output, pGain = 0.00, iGain = 0.00;

  glOmega = enc_val_right - enc_val_left;     //for omega 誤差(?)出るからパルス=>radは後で
  glDeg += glOmega;
  deg = glDeg * PULSE_TO_MM / TREAD * 180.00 / PI;

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

  Serial.println(Ycur);

  deff_deg = glDeg + acos((Yref-Ycur)/sqrt((Xref-Xcur)*(Xref-Xcur)+(Yref-Ycur)*(Yref-Ycur)));
  output = deff_deg * pGain + (SPEED_RIGHT_REF-velocity) * iGain;

}

void motorStraightRight(int pwm){
  int i;

  for(i = 0; i < PWM_MAX; i++){
    if(i<pwm){
      PORTG |= B00100000;
//      digitalWrite(4,HIGH);
    }
    else{
      PORTG &= ~B00100000;
//      digitalWrite(4,LOW);
    }
  }
}

void motorStraightLeft(int pwm){
  int i;
  for(i = 0; i < PWM_MAX; i++){
    if(i<pwm){
      PORTH |= B00001000;
//      digitalWrite(6, HIGH);
    }
    else{
      PORTH &= ~B00001000;
//      digitalWrite(6, LOW);
    }
  }
}


void loop(){

if(glDeg<0){
  kakunin1--;
  kakunin2++;
}
else if(glDeg>0){
  kakunin1++;
  kakunin2--;
}
motorStraightRight(30);
motorStraightLeft(30);
/*
  if(abs(Ycur)<500.00){
    analogWrite(4,30+kakunin2);
    analogWrite(5,0);
    analogWrite(6,30+kakunin1);
    analogWrite(7,0);
  }
  else{
    analogWrite(4,0);
    analogWrite(5,0);
    analogWrite(6,0);
    analogWrite(7,0);
  }
*/
}
