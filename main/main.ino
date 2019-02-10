#include <math.h>
#include <FlexiTimer2.h>
#include <parameters.h>



int lineCounter = 0;		    //linecount
int MaxLineCount = 4;
short flg = 0, preflg = 0;

volatile int enc_val_left = 0, enc_val_right = 0;	 //encoder
volatile uint8_t enc_prev_left = 0, enc_prev_right = 0;

void setup() {
  parameters();
// No.5 pin-setting for motor PWM
  TCCR3A = B10101001;
  TCCR3B = B00001011;
// No.6,7,8
  TCCR4A = B10101001;
  TCCR4B = B00001011;

  attachInterrupt(5, updateEncoder1, CHANGE);
  attachInterrupt(4, updateEncoder1, CHANGE);
  attachInterrupt(3, updateEncoder2, CHANGE);
  attachInterrupt(2, updateEncoder2, CHANGE);

//  FlexiTimer2::set(SPEEDCONTROL_TIME * 1000, odmetry);
//  FlexiTimer2::start();

  Serial.begin(9600);
}

void updateEncoder1() {
  //  uint8_t a = digitalRead(18);
  //  uint8_t b = digitalRead(19);

  uint8_t a, b;
  if (PIND & _BV(PD2)) {
    a = HIGH;
  }
  else {
    a = LOW;
  }
  if (PIND & _BV(PD3)) {
    b = HIGH;
  }
  else {
    b = LOW;
  }

  uint8_t ab = (a << 1) | b;
  uint8_t encoded  = (enc_prev_left << 2) | ab;

  if (encoded == 0b1101 || encoded == 0b0100 || encoded == 0b0010 || encoded == 0b1011) {
    enc_val_right ++;
  }
  else if (encoded == 0b1110 || encoded == 0b0111 || encoded == 0b0001 || encoded == 0b1000) {
    enc_val_right --;
  }

  enc_prev_right = ab;
}

void updateEncoder2() {
  //  uint8_t c = digitalRead(2);
  //  uint8_t d = digitalRead(3);
  uint8_t c, d;
  if (PIND & _BV(PE5)) {
    c = HIGH;
  }
  else {
    c = LOW;
  }
  if (PIND & _BV(PE4)) {
    d = HIGH;
  }
  else {
    d = LOW;
  }

  uint8_t cd = (c << 1) | d;
  uint8_t encoded  = (enc_prev_left << 2) | cd;

  if (encoded == 0b1101 || encoded == 0b0100 || encoded == 0b0010 || encoded == 0b1011) {
    enc_val_left ++;
  }
  else if (encoded == 0b1110 || encoded == 0b0111 || encoded == 0b0001 || encoded == 0b1000) {
    enc_val_left --;
  }

  enc_prev_left = cd;
}

void motorR(int PWM){
  if(PWM >= 0){
    OCR3A = 0;      //0~255(No.5pin)
    OCR4A = PWM;    //0~255(No.6pin)
  }
  else{
    OCR3A = -PWM;    //0~255(No.5pin)
    OCR4A = 0;       //0~255(No.6pin)
  }
}

void motorL(int PWM){
  if(PWM >= 0){
    OCR4B = PWM;     //0~255(No.7pin)
    OCR4C = 0;       //0~255(No.8pin)
  }
  else{
    OCR4B = 0;    //0~255(No.7pin)
    OCR4C = -PWM; //0~255(No.8pin)
  }
}

void straight(){

}

void lineCount(){
  if(digitalRead(A0) == HIGH || digitalRead(A4) == HIGH){
    flg = 1;
  }
  else{
    flg = 0;
  }

  if(flg - preflg == -1) lineCounter++;

  preflg = flg;
}

/*
void lineTrace() {
  int Ana0, Ana1, Ana2, Ana3;
  Ana0 = analogRead(A0);
  Ana1 = analogRead(A1);
  Ana2 = analogRead(A2);
  Ana3 = analogRead(A3);

  if (Ana1 < 100) {
    PWM_Right -= 40;
    PWM_Left += 20;
  }
  else if (Ana2 < 100) {
    PWM_Right += 20;
    PWM_Left -= 40;
  }
  else {
    speedRefRight = 100;
    speedRefLeft = 100;
  }
  speedControl();

  Serial.print(Ana0);
  Serial.print("	");
  Serial.print(Ana1);
  Serial.print("	");
  Serial.print(Ana2);
  Serial.print("	");
  Serial.println(Ana3);

}
*/

void searchBall(){

}

void catchMotion(){

}

void turnDeg(int deg){

}

void shootMotion(){

}

void initialMotion(){

}

void mvArea(int MaxLineCnt){

}

void collectBall(){

}

void getScore(){

}

/*
void odmetry() {
  double glOmega, glVelocity;
  double rad = 0.00;

  glOmega = enc_val_right - enc_val_left;     //for omega 誤差(?)出るからパルス=>radは後で
  glDeg += glOmega;
  Serial.println(glDeg);

  glVelocity = (enc_val_right + enc_val_left) / 2 * PULSE_TO_MM;
  rad = glDeg * PULSE_TO_MM / TREAD;
  Xcur += glVelocity * sin(rad);         //x-cordinate => "sin"
  Ycur += glVelocity * cos(rad);         //y-cordinate => "cos"

  speedControl();

  enc_val_right = 0;
  enc_val_left = 0;
}
*/

/*
void motorControl() {
  int i;
  int duty_Right = PWM_MAX / PWM_Right, duty_Left = PWM_MAX / PWM_Left;

  for (i = 1; i <= PWM_MAX; i++) {
    if ((i % duty_Right) == 0) {
      PORTE |= B00001000;
      //      digitalWrite(5,HIGH);
    }
    else {
      PORTE &= ~B0001000;
      //      digitalWrite(5,LOW);
    }

    if ((i % duty_Left) == 0) {
      PORTH |= B00010000;
      //      digitalWrite(7, HIGH);
    }
    else {
      PORTH &= ~B00010000;
      //      digitalWrite(7, LOW);
    }
  }
}

*/


void motorStop() {
  OCR3A = 0;
  OCR4A = 0;
  OCR4B = 0;
  OCR4C = 0;
}

/*
double low_pass_filter(double val, double pre_val, double gamma) {                             //gammma 限界値に近ずけるための定数
  return gamma * pre_val + (1 - gamma) * val;
}

void speedControl() {
  double deg;
  double speedRight = enc_val_right * PULSE_TO_MM / SPEEDCONTROL_TIME;
  double speedLeft = enc_val_left * PULSE_TO_MM / SPEEDCONTROL_TIME;

  double speedRight_lpf = 0.00, speedLeft_lpf = 0.00;
  speedRight_lpf = low_pass_filter(speedRight, speedRight_lpf, 0.4);
  speedLeft_lpf = low_pass_filter(speedLeft, speedLeft_lpf, 0.4);

  double diffSpeedRight = speedRefRight - speedRight_lpf;
  double diffSpeedLeft = speedRefLeft - speedLeft_lpf;

  deg = glDeg * PULSE_TO_MM / TREAD * 180.00 / PI;
//  Serial.println(deg);
//  deff_deg = deg + acos((Yref-Ycur)/sqrt((Xref-Xcur)*(Xref-Xcur)+(Yref-Ycur)*(Yref-Ycur)))*180/PI;

//  PWM_Right = diffSpeedRight * PGAIN_R;
//  PWM_Left = diffSpeedLeft * PGAIN_L;

  PWM_Right = diffSpeedRight * PGAIN_R + (diffSpeedRight * MOTOR_TIME) * IGAIN_R + glDeg * DGAIN_R;
  PWM_Left = diffSpeedLeft * PGAIN_L + (diffSpeedLeft * MOTOR_TIME) * IGAIN_L + glDeg * DGAIN_L;
  if (PWM_Right > 256) {
    PWM_Right = 256;
  }
  else if (PWM_Right <= 0) {
    PWM_Right = 1;
  }
  if (PWM_Left > 256) {
    PWM_Left = 256;
  }
  else if (PWM_Left <= 0) {
    PWM_Left = 1;
  }

  Serial.print(diffSpeedRight);
  Serial.print("	");
  Serial.println(PWM_Right);

  motorControl();

}
*/

void loop() {



}
