volatile long value1 = 0; //left
volatile long value2 = 0;
volatile uint8_t prev1 = 0;
volatile uint8_t prev2 = 0;

void updateEncoder1();
void updateEncoder2();

void setup() {
/*
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
*/
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  attachInterrupt(5, updateEncoder1, CHANGE); //interrupt pin18=5, pin19=4
  attachInterrupt(4, updateEncoder1, CHANGE);
  attachInterrupt(3, updateEncoder2, CHANGE);
  attachInterrupt(2, updateEncoder2, CHANGE);

//  Serial.begin(9600);
  //SetUpencoder();
}

void loop() {
/*  Serial.print(value1);
  Serial.print("  ");
  Serial.println(value2);
*/

  straight(value1, value2);
}

void updateEncoder1(){
  uint8_t a = digitalRead(18);
  uint8_t b = digitalRead(19);

 
  uint8_t ab = (a << 1) | b;
  uint8_t encoded1  = (prev1 << 2) | ab;

  if(encoded1 == 0b1101 || encoded1 == 0b0100 || encoded1 == 0b0010 || encoded1 == 0b1011){
    value1 ++;
  }
  else if(encoded1 == 0b1110 || encoded1 == 0b0111 || encoded1 == 0b0001 || encoded1 == 0b1000) {
    value1 --;
  }

  prev1 = ab;
}
void updateEncoder2(){
  uint8_t c = digitalRead(20);
  uint8_t d = digitalRead(21);
 
  uint8_t cd = (c << 1) | d;
  uint8_t encoded2  = (prev2 << 2) | cd;

  if(encoded2 == 0b1101 || encoded2 == 0b0100 || encoded2 == 0b0010 || encoded2 == 0b1011){
    value2 ++;
  }
  else if(encoded2 == 0b1110 || encoded2 == 0b0111 || encoded2 == 0b0001 || encoded2 == 0b1000) {
    value2 --;
  }

  prev2 = cd;  
}

void straight(long value1, long value2){
  int right=0, left=0;
  int Rspeed = 20, Lspeed = 20;
  if(value1<-116160){                             //116160=>pulse/rotate
    value1 += 116160;
    value2 += 116160;
  }
  if(value1>116160){
    value1 -= 116160;
    value2 -= 116160;
  }
  if(value1>value2){
    right++;
    left--;
  }
  else if(value1<value2){
    left++;
    right--;
  }
  else{
    right = 0;
    left = 0;
  }

  Rspeed += right;
  Lspeed += left;

  analogWrite(4,0);
  analogWrite(5,Rspeed);
  analogWrite(6,0);
  analogWrite(7,Lspeed);
}
