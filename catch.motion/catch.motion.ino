/*
  Analog input, analog output, serial output

  Reads an analog input pin, maps the result to a range from 0 to 255 and uses
  the result to set the pulse width modulation (PWM) of an output pin.
  Also prints the results to the Serial Monitor.

  The circuit:
  - potentiometer connected to analog pin 0.
    Center pin of the potentiometer goes to the analog pin.
    side pins of the potentiometer go to +5V and ground
  - LED connected from digital pin 9 to ground

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogInOutSerial
*/


// These constants won't change. They're used to give names to the pins used:
#include<Servo.h> 

const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to


float Vcc=5.0;
float dist;

Servo servo;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  servo.attach(9);
}

void loop() {
  // read the analog in value:
  dist =Vcc*analogRead(analogInPin)/1023;
  dist = 26.549*pow(dist,-1.2091);

  // print the results to the Serial Monitor:
  Serial.println("dist = ");
  Serial.print(dist);
  Serial.println("cm ");
  

  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  delay(1000);
  while(1){
    dist =Vcc*analogRead(analogInPin)/1023;
    dist = 26.549*pow(dist,-1.2091);

  
    Serial.println(dist);
    

  if(dist>=30&&dist<=40){
    servo.write(0);
  }
 else if(dist<=15){
    servo.write(90);
 }
 else if(dist>=40){
    servo.write(90);
 }
  }
  }
  






