#include <IcsSoftSerialClass.h>

const byte S_RX_PIN = 0;
const byte S_TX_PIN = 1;

const byte EN_PIN = 2;
const long BAUDRATE = 115200;
const int TIMEOUT =200;

IcsSoftSerialClass krs(S_RX_PIN,S_TX_PIN,EN_PIN,BAUDRATE,TIMEOUT);

void setup() {
  krs.begin();
}

void loop() {
  krs.setSpd(0,60);
  krs.setPos(0,7500);
  delay(500);
  krs.setPos(0,6500);
  delay(500);
  krs.setPos(0,5500);
  delay(500);
  
}
