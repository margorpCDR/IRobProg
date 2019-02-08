int linecount = 0;
int Max_linecount = 4;
short flg;
short pre_flg;
void setup() {
Serial.begin(9600);

}

void loop() {

if(digitalRead(A0) == HIGH || digitalRead(A4) == HIGH){
 flg = 1;
}
else{
 flg = 0;
}

if(flg - pre_flg != 0){
  linecount++;
}

pre_flg = flg;


/*Serial.print(linecount);                                        //確認用
Serial.print(flg);
Serial.print(pre_flg);
Serial.print("   ");
Serial.print(digitalRead(A0));
Serial.print("   ");
Serial.print(analogRead(A1));
Serial.print("   ");
Serial.print(analogRead(A2));
Serial.print("   ");
Serial.print(analogRead(A3));
Serial.print("   ");
Serial.println(digitalRead(A4));*/

}


