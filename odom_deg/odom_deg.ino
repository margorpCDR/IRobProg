#include<stdio.h>
#include<math.h>
#include <FlexiTimer2.h>
#define TREAD 190
#define PULSE_TO_MM 0.01586662956

long rEnc = 0.0;
long rEncoder =0.0;
long lEnc =0.0;
long lEncoder =0.0;
int  glX=0.0, glY=0.0;

volatile int value1 = 0, value2 = 0;
volatile uint8_t prev1 = 0, prev2 = 0;

double glOmega, glVelocity, glRho, glTheta, glDeg;
double glAnalogVelocity;

void setup() {
  
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

  FlexiTimer2::set(1, odometry); // 500ms period
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

void odometry(){
  
 

glOmega = ((value1 - value2)*PULSE_TO_MM)/TREAD;
glVelocity = ((value1 + value2)*PULSE_TO_MM) / 2;
//glRho = (TREAD/2) * ((value1*PULSE_TO_MM) + (value2*PULSE_TO_MM)) / ((value1*PULSE_TO_MM) - (value2*PULSE_TO_MM));


 
glTheta += glOmega;   
                                             
//glAnalogVelocity = (rEncoder + lEncoder) / 2;
/*if(glTheta < -PI) glTheta += (2.0*PI);      //-π～π
  if(glTheta > PI) glTheta -= (2.0*PI);*/
  
  glX += glVelocity * cos(glTheta);         //x座標
  glY += glVelocity * sin(glTheta);         //y座標
  glDeg = glTheta * 180 / PI *2;             //rad to deg ２倍するとうまくいく

 rEncoder +=value1;
 value1 =0;
 lEncoder +=value2;
 value2=0;
  
  }

void loop() {

  analogWrite(6,0);
  analogWrite(7,30);
  analogWrite(8,0);
  analogWrite(11,30);

if(glDeg<=-90){
  analogWrite(6,0);
  analogWrite(7,0);
  analogWrite(8,0);
  analogWrite(11,0);}
   

 
 
Serial.print("glOmega");
Serial.print("  ");
Serial.print(value2);
Serial.print("    ");
Serial.print("glTheta");
Serial.print("  ");
Serial.print(glTheta);   
Serial.print("    ");
Serial.print("glDeg");
Serial.print("  ");
Serial.println(glDeg);



}

