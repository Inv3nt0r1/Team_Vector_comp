//hatY = 0 -> UPside
//hatY = 255 -> DOWNside
//hatX = 0 -> LEFTside
//hatX = 255 -> RIGHTside
//dir = HIGH -> Clockwise
//dir = LOW -> AntiClockwise
//leftHatX = 0
//leftHatY = 1
//rightHatX = 2
//rightHatY = 5

#include "USBHost_t36.h"

//----------------Locomotion-------------------
#define lefttop_pwm 2
#define leftbottom_pwm 3
#define righttop_pwm 4
#define rightbottom_pwm 5
#define lefttop_dir 22
#define leftbottom_dir 23
#define righttop_dir 24
#define rightbottom_dir 25
//--------------------------------------------

//----------------Pneumatics------------------
#define pneu_up 29
#define pneu_down 30
//--------------------------------------------

USBHost Usb;
USBHub hub1(Usb);
USBHub hub2(Usb);
USBHub hub3(Usb);
USBHIDParser hid1(Usb);
USBHIDParser hid2(Usb);
USBHIDParser hid3(Usb);
USBHIDParser hid4(Usb);
USBHIDParser hid5(Usb);
JoystickController joystick1(Usb);

int Speed = 30;
String button1;
void setup() 
{
  Serial.begin(9600);
  Serial.println("USB Host Testing");
  Usb.begin();  
  //---------Initialize motors----------
  pinMode(lefttop_pwm,OUTPUT);
  pinMode(leftbottom_pwm,OUTPUT);
  pinMode(righttop_pwm,OUTPUT);
  pinMode(rightbottom_pwm,OUTPUT);
  pinMode(lefttop_dir,OUTPUT);
  pinMode(leftbottom_dir,OUTPUT);
  pinMode(righttop_dir,OUTPUT);
  pinMode(rightbottom_dir,OUTPUT); 
  //-------------------------------------

  //--------Initialize pneumatics--------
  pinMode(pneu_up,OUTPUT);
  pinMode(pneu_down,OUTPUT);
  //-------------------------------------
}

void loop()
{
    Usb.Task();
    if (joystick1.available())
    {
      button1=String(joystick1.getButtons(),HEX);
      //Serial.println("Serial available");
      if(button1==200) // pneumatics up
      {
                  digitalWrite(pneu_up,HIGH);
                  Serial.println("pneumatics up");
                  delay(100);
                  digitalWrite(pneu_up,LOW);
      }
      if(button1==100) // pneumatics down
      {
                  digitalWrite(pneu_down,HIGH);
                  Serial.println("pneumatics down");
                  delay(100);
                  digitalWrite(pneu_down,LOW);
       }
       if(button1==800) // increase speed of motor
       {
                  Speed=(Speed>254?255:Speed+1); 
                  delay(100);
                  Serial.println("wheels_speed : ");
                  Serial.println(Speed);
       }
       else if(button1==400) // decrease speed of motor
       {
                  Speed=(Speed<1?0:Speed-1);
                  delay(100);
                  Serial.println("wheels_speed : ");
                  Serial.println(Speed);
       }
       locomotion();
    }
    else
    {
       Serial.println("Joystick not available");
    }
}
void locomotion()
{
       if(joystick1.getAxis(0)==255 && joystick1.getAxis(2)==0)
       {
        analogWrite(lefttop_pwm,0);
        analogWrite(leftbottom_pwm,0);
        analogWrite(rightbottom_pwm,0);   
        analogWrite(righttop_pwm,0);
       }
       else
       if(joystick1.getAxis(0)==0 && joystick1.getAxis(2)==255)
       {
        analogWrite(lefttop_pwm,0);
        analogWrite(leftbottom_pwm,0);
        analogWrite(rightbottom_pwm,0);   
        analogWrite(righttop_pwm,0);
       }
       else
       if(joystick1.getAxis(0)==255 && joystick1.getAxis(2)==255)
       {
           digitalWrite(lefttop_dir,HIGH);
           digitalWrite(leftbottom_dir,HIGH);
           digitalWrite(righttop_dir,HIGH);
           digitalWrite(rightbottom_dir,LOW);
           Serial.println("bot right");
           analogWrite(lefttop_pwm,Speed);   
           analogWrite(leftbottom_pwm,Speed); 
           analogWrite(righttop_pwm,Speed);   
           analogWrite(rightbottom_pwm,Speed); 
       }
       else
       if(joystick1.getAxis(0)==0 && joystick1.getAxis(2)==0)
       {
           digitalWrite(lefttop_dir,LOW);
           digitalWrite(leftbottom_dir,LOW);
           digitalWrite(righttop_dir,LOW);
           digitalWrite(rightbottom_dir,HIGH);
           Serial.println("bot left");
           analogWrite(lefttop_pwm,Speed);   
           analogWrite(leftbottom_pwm,Speed);  
           analogWrite(righttop_pwm,Speed);   
           analogWrite(rightbottom_pwm,Speed);
       }
       else
       if(joystick1.getAxis(1)==255 && joystick1.getAxis(5)==0) 
       {
        digitalWrite(lefttop_dir,LOW);
        digitalWrite(leftbottom_dir,HIGH);
        digitalWrite(righttop_dir,LOW);
        digitalWrite(rightbottom_dir,LOW);
        Serial.println("bot anticlock");
        analogWrite(lefttop_pwm,Speed);
        analogWrite(leftbottom_pwm,Speed);
        analogWrite(rightbottom_pwm,Speed);   
        analogWrite(righttop_pwm,Speed);
       }
       else 
       if(joystick1.getAxis(1)==0 && joystick1.getAxis(5)==255)
       {
        digitalWrite(lefttop_dir,HIGH);
        digitalWrite(leftbottom_dir,LOW);
        digitalWrite(righttop_dir,HIGH);
        digitalWrite(rightbottom_dir,HIGH);
        Serial.println("bot clock");
        analogWrite(lefttop_pwm,Speed);   
        analogWrite(leftbottom_pwm,Speed);
        analogWrite(rightbottom_pwm,Speed);
        analogWrite(righttop_pwm,Speed);
       }
       else 
       if(joystick1.getAxis(1)==0 && joystick1.getAxis(5)==0)
       {
        digitalWrite(lefttop_dir,HIGH);
        digitalWrite(leftbottom_dir,LOW);
        digitalWrite(righttop_dir,LOW);
        digitalWrite(rightbottom_dir,LOW);
        Serial.println("bot front");
        analogWrite(lefttop_pwm,Speed);   
        analogWrite(leftbottom_pwm,Speed);
        analogWrite(rightbottom_pwm,Speed);
        analogWrite(righttop_pwm,Speed);
       }
       else 
       if(joystick1.getAxis(1)==255 && joystick1.getAxis(5)==255)
       {
        digitalWrite(lefttop_dir,LOW);
        digitalWrite(leftbottom_dir,HIGH);
        digitalWrite(righttop_dir,HIGH);
        digitalWrite(rightbottom_dir,HIGH);
        Serial.println("bot back");
        analogWrite(lefttop_pwm,Speed);   
        analogWrite(leftbottom_pwm,Speed);
        analogWrite(rightbottom_pwm,Speed);
        analogWrite(righttop_pwm,Speed);
       }
       else
       {
        
         if(joystick1.getAxis(1)==255)
         {
          digitalWrite(lefttop_dir,LOW);
          digitalWrite(leftbottom_dir,HIGH);
          Serial.println("bot left down");
          analogWrite(lefttop_pwm,Speed);   
          analogWrite(leftbottom_pwm,Speed);  
         }
         else
         if(joystick1.getAxis(1)==0)
         {
          digitalWrite(lefttop_dir,HIGH);
          digitalWrite(leftbottom_dir,LOW);
          Serial.println("bot left up");
          analogWrite(lefttop_pwm,Speed);   
          analogWrite(leftbottom_pwm,Speed);   
         } 
         else
         if(joystick1.getAxis(0)==255)
         {
           digitalWrite(lefttop_dir,HIGH);
           digitalWrite(leftbottom_dir,HIGH);
           digitalWrite(righttop_dir,HIGH);
           digitalWrite(rightbottom_dir,LOW);
           Serial.println("bot right");
           analogWrite(lefttop_pwm,Speed);   
           analogWrite(leftbottom_pwm,Speed); 
           analogWrite(righttop_pwm,Speed);   
           analogWrite(rightbottom_pwm,Speed);  
         } 
         else
         if(joystick1.getAxis(0)==0)
         {
           digitalWrite(lefttop_dir,LOW);
           digitalWrite(leftbottom_dir,LOW);
           digitalWrite(righttop_dir,LOW);
           digitalWrite(rightbottom_dir,HIGH);
           Serial.println("bot left");
           analogWrite(lefttop_pwm,Speed);   
           analogWrite(leftbottom_pwm,Speed);  
           analogWrite(righttop_pwm,Speed);   
           analogWrite(rightbottom_pwm,Speed);  
         }
         else
         if(joystick1.getAxis(5)==255)
         {
           digitalWrite(righttop_dir,HIGH);
           digitalWrite(rightbottom_dir,HIGH);
           Serial.println("bot right down");
           analogWrite(righttop_pwm,Speed);
           analogWrite(rightbottom_pwm,Speed);
         }
         else
         if(joystick1.getAxis(5)==0)
         {
           digitalWrite(righttop_dir,LOW);
           digitalWrite(rightbottom_dir,LOW);
           Serial.println("bot right up");
           analogWrite(righttop_pwm,Speed);
           analogWrite(rightbottom_pwm,Speed);
         }
         else
         {
           analogWrite(righttop_pwm,0);
           analogWrite(rightbottom_pwm,0);
           analogWrite(lefttop_pwm,0);
           analogWrite(leftbottom_pwm,0);
         }
       }
}
