#include "arduino.h"
#include "parameters.h"

parameters::parameters(){
//encoder setting
  DDRE &= ~B00001000;
  DDRD &= ~B00001100;
  /*
  pinMode(2, INPUT);        //encoder l
  pinMode(3, INPUT);
  pinMode(18 , INPUT);        //encoder r
  pinMode(19 , INPUT);*/

//motor setting

  DDRE |= _BV(PE3);
  DDRH |= B00111000;
  /*
  pinMode(5, OUTPUT);         //motorL  +
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);         //motorR  -
  pinMode(8, OUTPUT);*/



}
