#include<Servo.h>
Servo ser1;
Servo ser2;
Servo ser3;
Servo ser4;
Servo ser5;
Servo ser6;
Servo ser7;
Servo ser8;
int pos1=90;
int pos2=90;
int pos3=90;
int pos4=90;
int pos5=90;
int pos6=90;
int pos7=90;
int pos8=90;
void setup()
{
  ser1.attach(2);
  ser2.attach(3);
  ser3.attach(4);
  ser4.attach(5);
  ser5.attach(6);
  ser6.attach(7);
  ser7.attach(8);
  ser8.attach(9);
  ser1.write(90);
  ser2.write(90);
  ser3.write(90);
  ser4.write(90);
  ser5.write(90);
  ser6.write(90); 
  ser7.write(90);
  ser8.write(90);
   Serial.begin(9600);
}
void loop()
{ 
  //legfr();
  //legbr();
  legfl();
  //legbl();
}

void legfr()
{
  while(pos1<=160)
  {
    servo_rotation(ser1,pos1);
    pos1++;
    servo_rotation(ser2,pos2);
    if(pos2<=110)
    pos2++;
    Serial.print("Pos1:");
    Serial.println(pos1);
    Serial.print("Pos2:");
    Serial.println(pos2);
  } 
  delay(1000);
  move_servo(ser2,110,90); 
  move_servo(ser1,160,90);
  delay(1000);
  pos1=90;
  pos2=90;
}
void legbr()
{
 while(pos3<=160)
  {
    servo_rotation(ser3,pos3);
    pos3++;
    servo_rotation(ser4,pos4);
    if(pos4<=110)
    pos4++;
    Serial.print("Pos3:");
    Serial.println(pos3);
    Serial.print("Pos4:");
    Serial.println(pos4);
  }  
  move_servo(ser4,110,90);
  move_servo(ser3,160,90);
  delay(1000);
  pos3=90;
  pos4=90; 
 }
 void legfl()
 {
  while(pos5>=20)
  {
    servo_rotation(ser5,pos5);
    pos5--;
    servo_rotation(ser6,pos6);
    if(pos6>=70)
    pos6--;
    Serial.print("Pos5:");
    Serial.println(pos5);
    Serial.print("Pos6:");
    Serial.println(pos6);
  }  
  move_servo(ser6,70,90);
  move_servo(ser5,20,90);
  delay(1000);
  pos6=90;
  pos5=90;
 }
 void legbl()
  {
    while(pos7>=20)
  {
    servo_rotation(ser7,pos7);
    pos7--;
    servo_rotation(ser8,pos8);
    if(pos8>=70)
    pos8--;
    Serial.print("Pos7:");
    Serial.println(pos7);
    Serial.print("Pos8:");
    Serial.println(pos8);
  }
 
 move_servo(ser8,70,90); 
 move_servo(ser7,20,90);
 delay(1000);
 pos7=90;
 pos8=90;
}
void move_servo(Servo s,int from_degree,int to_degree)
{
  if(from_degree<to_degree)
  {
    for(int i=from_degree;i<=to_degree;i++)
    {
      ///*
      s.write(i);
      Serial.print("i:");
      Serial.println(i);
      delayMicroseconds(500);
      //*/
      //servo_rotation(s,i);
    }
  }
  else if(from_degree>to_degree)
  {
    for(int i=from_degree;i>=to_degree;i--)
    {
     // /*
     s.write(i);
     Serial.print("i:");
     Serial.println(i);
     delayMicroseconds(500);
   //*/
      //servo_rotation(s,i);
    }
  }
 delay(1000); 
}
void servo_rotation(Servo sr,int pos)
{
  sr.write(pos);
  Serial.print("Pos:");
  Serial.println(pos);
  delayMicroseconds(500);  
}
void sit()
{
  ser5.write(135);
  ser6.write(45);
}

/*void ser1rotation(int j)
{
  ser1.write(j);
  delayMicroseconds(500);
}

void ser2rotation(int i)
{
  ser2.write(i);
  delayMicroseconds(500);
}
void ser3rotation(int j)
{
  ser3.write(j);
  delayMicroseconds(500);
}
void ser4rotation(int j)
{
  ser4.write(j);
  delayMicroseconds(500);
}
void ser5rotation(int j)
{
  ser5.write(j);
  delayMicroseconds(500);
}
void ser6rotation(int j)
{
  ser6.write(j);
  delayMicroseconds(500);
}
void ser7rotation(int j)
{
  ser7.write(j);
  delayMicroseconds(500);
}
void ser8rotation(int j)
{
  ser8.write(j);
  delayMicroseconds(500);
}*/
