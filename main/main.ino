#include<math.h>
#include <FlexiTimer2.h>
#include <parameters.h>

#define MOTOR_TIME 0.2
#define SPEEDCONTROL_TIME 0.01

volatile int enc_val_right = 0, enc_val_left = 0;
volatile uint8_t enc_prev_right = 0, enc_prev_left = 0;
double glDeg = 0.00;
double  Xcur=0.00, Ycur=0.00;
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

  FlexiTimer2::set(SPEEDCONTROL_TIME * 1000, speedControl);
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

void speedControl(){
  double Xref = 0.00, Yref = 100.00;
  double glOmega = 0.00, glVelocity = 0.00, glRho = 0.00, glTheta = 0.00;
  double rad = 0.00, deg = 0.00, deff_deg = 0.00;
  double pGainRight = 4.00, iGainRight = 0.30, dGainRight = 0.0;
  double pGainLeft = 2.55, iGainLeft = 0.20, dGainLeft = 0.0;

  glOmega = enc_val_right - enc_val_left;     //for omega 誤差(?)出るからパルス=>radは後で
  glDeg += glOmega;
  deg = glDeg * PULSE_TO_MM / TREAD * 180.00 / PI;
  deg *= 1.125;                               //miracle magic number

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

//  deff_deg = deg + acos((Yref-Ycur)/sqrt((Xref-Xcur)*(Xref-Xcur)+(Yref-Ycur)*(Yref-Ycur)))*180/PI;
  if(flg == 1){
//    kakunin1 = deff_deg * pGainRight - glOmega * iGainRight + (Yref-Ycur) * dGainRight;
//    kakunin2 = -deff_deg * pGainLeft + glOmega * iGainLeft + (Yref-Ycur) * dGainLeft;
//    kakunin1 = -deff_deg * pGainRight;
//    kakunin2 = +deff_deg * pGainLeft;

      kakunin1 = glOmega * pGainRight + (Yref-Ycur) * iGainRight;
      kakunin2 = glOmega * pGainLeft + (Yref-Ycur) * iGainLeft;
    if(kakunin1 > 256){
      kakunin1 = 256;
    }
    else if(kakunin1 <= 0){
      kakunin1 = 1;
    }
    if(kakunin2 > 256){
      kakunin2 = 256;
    }
    else if(kakunin2 <= 0){
      kakunin2 = 1;
    }
  }
  flg = 0;
  if(Yref <= Ycur){
    Yref += 100;
  }
//Serial.println(kakunin2);
  Serial.println(glOmega);
}

void goStraight(int PWM_Right, int PWM_Left){
  int i;
  int motor_Right = PWM_MAX/PWM_Right, motor_Left = PWM_MAX/PWM_Left;
  for(i = 1; i <= PWM_MAX; i++){
    if((i%motor_Right) == 0){
      PORTG |= B00100000;
//      digitalWrite(4,HIGH);
    }
    else{
      PORTG &= ~B00100000;
//      digitalWrite(4,LOW);
    }

    if((i%motor_Left) == 0){
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

void loop(){
  if(Ycur>-50000.00){
    goStraight(kakunin1, kakunin2);
  }
  else{
    motorStop();
  }
}
