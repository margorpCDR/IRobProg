#include<Servo.h>
#include <IcsSoftSerialClass.h>

const int analogInPin = A1;
float Vcc=5.0;
float dist;
int pos = 90;

/**     servo 関係        **/
const byte S_TX_PIN = 1;
const long BAUDRATE = 115200;
const int TIMEOUT =200;
IcsSoftSerialClass krs(S_TX_PIN,BAUDRATE,TIMEOUT);
Servo yow;


void setup() {
  
pinMode(38,OUTPUT);
yow.attach(12);
krs.begin();

}

void loop() {

//digitalWrite(38,HIGH);

for(pos = 50; pos<= 140; pos+=1){ 
yow.write(pos);
dist =Vcc *analogRead(analogInPin)/1023;
dist = 26.549*pow(dist,-1.2091);
delay(50);

/*if(dist > 10) {  //腕を伸ばすだけで吸い込める距離
  krs.setSpd(0,60);
  krs.setPos(0,6000); 
}

 else if(dist > 16 && dist <30 ) {  //腕を伸ばすだけで吸い込める距離
  krs.setSpd(0,60);
  krs.setPos(0,7500); 
}

else{
krs.setSpd(0,60);
krs.setPos(0,5500); 

}
Serial.println(dist);*/
}

for(pos = 140; pos >= 50; pos -= 1){
yow.write(pos);
dist =Vcc *analogRead(analogInPin)/1023;
dist = 26.549*pow(dist,-1.2091);
delay(50);


Serial.println(dist); 

}






}
