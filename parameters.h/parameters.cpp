#include "arduino.h"
#include "parameters.h"

parameters::parameters(){
//encoder setting
//  DDRD &= ~B00000000;

  pinMode(2, INPUT);        //encoder r
  pinMode(3, INPUT);
  pinMode(18 , INPUT);        //encoder l
  pinMode(19 , INPUT);

//motor setting

  /*DDRG |= _BV(PG5);
  DDRE |= _BV(PE3);
  DDRH |= B00011000;/*
  /*
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(11, OUTPUT);
*/

}
