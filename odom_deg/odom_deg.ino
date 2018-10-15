#include<math.h>
#include <FlexiTimer2.h>

#define TREAD 294
#define PULSE_TO_MM 0.0021365858
#define ODOMETRY_TIME 0.01

volatile int enc_val_right = 0, enc_val_left = 0;
volatile uint8_t enc_prev_right = 0, enc_prev_left = 0;
double glDeg = 0.0000;
long distance_right = 0.0000, distance_left = 0.0000;



void setup() {
//encoder setting
  pinMode(18, INPUT);        //encoder r
  pinMode(19, INPUT);
  pinMode(20,INPUT);        //encoder l
  pinMode(21,INPUT);
  attachInterrupt(5, updateEncoder1, CHANGE);
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
  digitalWrite(18,HIGH);
  digitalWrite(19,HIGH);

  FlexiTimer2::set(ODOMETRY_TIME * 1000, odometry);
  FlexiTimer2::start();
  
  Serial.begin(9600);
}


void updateEncoder1(){
  uint8_t a = digitalRead(18);
  uint8_t b = digitalRead(19);
 
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
  uint8_t c = digitalRead(20);
  uint8_t d = digitalRead(21);
 
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
  int  glX=0.0000, glY=0.0000;
  double glOmega = 0.0000, glVelocity = 0.0000, glRho = 0.0000, glTheta = 0.0000;
  
  glOmega = ((enc_val_right - enc_val_left) * PULSE_TO_MM) / TREAD;     //Omega[rad/sec]
  glVelocity = ((enc_val_right + enc_val_left) * PULSE_TO_MM) / 2;
//  glRho = glOmega / glVelocity;
 
  glTheta += glOmega;

  glX += glVelocity * cos(glTheta);         //x座標
  glY += glVelocity * sin(glTheta);         //y座標
  glDeg += glTheta * 180.00 / PI;             //rad to deg

  distance_right += enc_val_right * PULSE_TO_MM;
  distance_left += enc_val_left * PULSE_TO_MM;
  enc_val_right = 0;
  enc_val_left = 0;

  Serial.println(glDeg);

}

void loop() {

  if(abs(glDeg)<=90){
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

//  Serial.println(glDeg);

}
