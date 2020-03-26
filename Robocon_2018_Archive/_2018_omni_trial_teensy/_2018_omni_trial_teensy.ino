//hatY = 0 -> UPside
//hatY = 255 -> DOWNside
//hatX = 0 -> LEFTside
//hatX = 255 -> RIGHTside
//dir = HIGH -> Clockwise
//dir = LOW -> AntiClockwise


#include "USBHost_t36.h"

//----------------Locomotion-------------------
#define left_pwm 2
#define right_pwm 3
#define up_pwm 4
#define down_pwm 5
#define left_dir 22
#define right_dir 23
#define up_dir 24
#define down_dir 25
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

int Speed = 25;
String button1;

void setup() 
{
  Serial.begin(9600);
  Serial.println("USB Host Testing");
  Usb.begin(); 
    
  //---------Initialize motors----------
  pinMode(left_pwm,OUTPUT);
  pinMode(right_pwm,OUTPUT);
  pinMode(up_pwm,OUTPUT);
  pinMode(down_pwm,OUTPUT);
  pinMode(left_dir,OUTPUT);
  pinMode(right_dir,OUTPUT);
  pinMode(up_dir,OUTPUT);
  pinMode(down_dir,OUTPUT); 
  //-------------------------------------
}

void loop()
{
    Usb.Task();
    
    if (joystick1.available())
    {
            button1=String(joystick1.getButtons(),HEX);
            Serial.println(button1);
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
        analogWrite(left_pwm,0);
        analogWrite(right_pwm,0);
        analogWrite(up_pwm,0);   
        analogWrite(down_pwm,0);
       }
       else
       if(joystick1.getAxis(0)==0 && joystick1.getAxis(2)==255)
       {
        analogWrite(left_pwm,0);
        analogWrite(right_pwm,0);
        analogWrite(up_pwm,0);   
        analogWrite(down_pwm,0);
       }
       else
       if(joystick1.getAxis(0)==255 && joystick1.getAxis(2)==255)
       {
         digitalWrite(up_dir,HIGH);
         digitalWrite(down_dir,HIGH);
         Serial.println("bot right");
         analogWrite(up_pwm,Speed);
         analogWrite(down_pwm,Speed);
       }
       else
       if(joystick1.getAxis(0)==0 && joystick1.getAxis(2)==0)
       {
         digitalWrite(up_dir,LOW);
         digitalWrite(down_dir,LOW);
         Serial.println("bot left");
         analogWrite(up_pwm,Speed);
         analogWrite(down_pwm,Speed);
       }
       else
       if(joystick1.getAxis(1) == 255 && joystick1.getAxis(5) == 0) //
       {
         digitalWrite(up_dir,LOW);
         digitalWrite(down_dir,HIGH);
         digitalWrite(left_dir,HIGH);
         digitalWrite(right_dir,HIGH);
         Serial.println("bot anticlock");
         analogWrite(up_pwm,Speed);
         analogWrite(down_pwm,Speed);
         analogWrite(left_pwm,Speed);
         analogWrite(right_pwm,Speed);
       }
       else 
       if(joystick1.getAxis(1) == 0 && joystick1.getAxis(5) == 255)
       {
         digitalWrite(up_dir,HIGH);
         digitalWrite(down_dir,LOW);
         digitalWrite(left_dir,LOW);
         digitalWrite(right_dir,LOW);
         Serial.println("bot clock");
         analogWrite(up_pwm,Speed);
         analogWrite(down_pwm,Speed);
         analogWrite(left_pwm,Speed);
         analogWrite(right_pwm,Speed);
       }
       else 
       if(joystick1.getAxis(1) == 0 && joystick1.getAxis(5) == 0)
       {
         digitalWrite(left_dir,LOW);
         digitalWrite(right_dir,HIGH);
         Serial.println("bot front");
         analogWrite(left_pwm,Speed);
         analogWrite(right_pwm,Speed);
       }
       else 
       if(joystick1.getAxis(1) == 255 && joystick1.getAxis(5) == 255)
       {
         digitalWrite(left_dir,HIGH);
         digitalWrite(right_dir,LOW);
         Serial.println("bot back");
         analogWrite(left_pwm,Speed);
         analogWrite(right_pwm,Speed);
       }
       else
       {
        
         if(joystick1.getAxis(1) == 255)
         {
            digitalWrite(left_dir,HIGH);
            Serial.println("bot left down");
            analogWrite(left_pwm,Speed);
         }
         else
         if(joystick1.getAxis(1) == 0)
         {
            digitalWrite(left_dir,LOW);
            Serial.println("bot left up");
            analogWrite(left_pwm,Speed);
         } 
         else
         if(joystick1.getAxis(0) == 255)
         {
           digitalWrite(up_dir,HIGH);
           digitalWrite(down_dir,HIGH);
           Serial.println("bot right");
           analogWrite(up_pwm,Speed);
           analogWrite(down_pwm,Speed);
         } 
         else
         if(joystick1.getAxis(0) == 0)
         {
           digitalWrite(up_dir,LOW);
           digitalWrite(down_dir,LOW);
           Serial.println("bot left");
           analogWrite(up_pwm,Speed);
           analogWrite(down_pwm,Speed);        
         }
         else
         if(joystick1.getAxis(5) == 255)
         {
           digitalWrite(right_dir,LOW);
           Serial.println("bot right down");
           analogWrite(right_pwm,Speed);
         }
         else
         if(joystick1.getAxis(5) == 0)
         {
           digitalWrite(right_dir,HIGH);
           Serial.println("bot right up");
           analogWrite(right_pwm,Speed);
         }
         else
         {
          analogWrite(left_pwm,0);
          analogWrite(right_pwm,0);
          analogWrite(up_pwm,0);   
          analogWrite(down_pwm,0);
         }
       }
}
