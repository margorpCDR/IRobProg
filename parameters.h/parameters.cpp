#include "arduino.h"
#include "parameters.h"

parameters::parameters(){
//encoder setting
//  DDRD &= ~B00000000;

  pinMode(2, INPUT);        //encoder l
  pinMode(3, INPUT);
  pinMode(18 , INPUT);      //encoder r
  pinMode(19 , INPUT);

//motor setting

  /*DDRG |= _BV(PG5);
  DDRE |= _BV(PE3);
  DDRH |= B00011000;*/
  
  pinMode(5, OUTPUT);      //Left motor
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);      //Right motor
  pinMode(8, OUTPUT);
  


}
