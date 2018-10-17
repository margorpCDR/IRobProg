#include<math.h>
#include <FlexiTimer2.h>
#include <parameters.h>

#define MOTOR_TIME 0.2
#define ODOMETRY_TIME 0.01

volatile int enc_val_right = 0, enc_val_left = 0;
volatile uint8_t enc_prev_right = 0, enc_prev_left = 0;
double glDeg = 0.00;
long distance_right = 0.00, distance_left = 0.00;

double kakunin=0;



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
  double Xcur=0.00, Ycur=0.00, Xref = 0.00, Yref = 0.00;
  double glOmega = 0.00, glVelocity = 0.00, glRho = 0.00, glTheta = 0.00;
  double rad = 0.00, deg = 0.00, deff_deg = 0.00, velocity = 0.00;
  double gain;

  glOmega = enc_val_right - enc_val_left;     //for omega 誤差(?)出るからパルス=>radは後で
  glVelocity = (enc_val_right + enc_val_left) / 2;
//  glRho = glOmega / glVelocity;

  glTheta = glOmega;
  rad = glTheta * PULSE_TO_MM;
  velocity = glVelocity * PULSE_TO_MM;

  Xcur += velocity * cos(rad);         //x座標
  Ycur += velocity * sin(rad);         //y座標
  glDeg += glTheta;


  distance_right += enc_val_right;    //後でPULSE_TO_MM計算
  distance_left += enc_val_left;
  enc_val_right = 0;
  enc_val_left = 0;

  deg = glDeg * PULSE_TO_MM / TREAD * 180.00 / PI;
  Serial.println(deg);

  deff_deg = glDeg + acos((Yref-Ycur)/sqrt((Xref-Xcur)*(Xref-Xcur)+(Yref-Ycur)*(Yref-Ycur)));
  gain = deff_deg * VPGAIN_r + (SPEED_RIGHT_REF-velocity) * VIGAIN_r;

}

void loop(){

  if(abs(glDeg)*PULSE_TO_MM/TREAD*180/PI<=90){
    analogWrite(4,0);
    analogWrite(5,30);
    analogWrite(6,30);
    analogWrite(7,0);
  }
  else{
    analogWrite(4,0);
    analogWrite(5,0);
    analogWrite(6,0);
    analogWrite(7,0);
  }

}
