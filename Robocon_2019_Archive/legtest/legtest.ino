#include<Servo.h>
#define hip 9
#define knee 10
Servo Hip;
Servo Knee;
void setup() 
{
 // put your setup code here, to run once:
  /*
  pinModehip,OUTPUT);
  pinMode(knee,OUTPUT);
  */
  Hip.attach(9);
  Knee.attach(10);
  Serial.begin(9600);
}

void loop() 
{/*
 while(1)
  {
    Hip.write(0);
    Knee.write(0);
    delay(25);
  }*/
 for(int i=0;i<70;i++)
 {
  Hip.write(i);
  Knee.write(i);
  delay(50);
 }
  while(1);
/*
  // put your main code here, to run repeatedly:
  for(int i=0;i<60;i++)
  {
    if(i > 20)
      Hip.write(i-20);
    if(i<40)
      Knee.write(40-i);
    delay(25);
    Serial.print("Hip: ");
    Serial.print(i);
  }
  for(int i=39;i>=0;i--)
  {
    Hip.write(i);
    Knee.write(40-i); 
    delay(25);
    Serial.print("Hip: ");
    Serial.print(i);
  }*/
}
