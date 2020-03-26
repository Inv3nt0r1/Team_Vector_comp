//hatY = 0 -> UPside
//hatY = 255 -> DOWNside
//hatX = 0 -> LEFTside
//hatX = 255 -> RIGHTside
//dir = HIGH -> Clockwise
//dir = LOW -> AntiClockwise

#include <PS3BT.h>
#include <usbhub.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

//----------------Locomotion-------------------
#define left_pwm 13
#define right_pwm 12
#define up_pwm 11
#define down_pwm 9
#define left_dir 22
#define right_dir 23
#define up_dir 24
#define down_dir 25
//--------------------------------------------

USB Usb;
BTD Btd(&Usb);

PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); 

int Speed = 25;

void setup() 
{
  Serial.begin(9600);
  ps3init(); 
    
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
    if (PS3.getButtonClick(PS)) 
    {
        Serial.print(F("\r\nPS"));
        PS3.disconnect();
    }
    else
    {
        if (PS3.PS3Connected || PS3.PS3NavigationConnected)
        {
            if(PS3.getButtonClick(R1)) // increase speed of motor
            {
                Speed=(Speed>250?255:Speed+5); 
                Serial.println("wheels_speed : ");
                Serial.println(Speed);
            }
            else if(PS3.getButtonClick(L1)) // decrease speed of motor
            {
                Speed=(Speed<5?0:Speed-5);
                Serial.println("wheels_speed : ");
                Serial.println(Speed);
            }
            locomotion();
        }
    }
}

void ps3init()
{
  #if !defined(__MIPSEL__)
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  #endif
  if (Usb.Init() == -1) 
  {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}

void locomotion()
{
       if(PS3.getAnalogHat(LeftHatX) == 255 && PS3.getAnalogHat(RightHatX) == 0)
       {
        analogWrite(left_pwm,0);
        analogWrite(right_pwm,0);
        analogWrite(up_pwm,0);   
        analogWrite(down_pwm,0);
       }
       else
       if(PS3.getAnalogHat(LeftHatX) == 0 && PS3.getAnalogHat(RightHatX) == 255)
       {
        analogWrite(left_pwm,0);
        analogWrite(right_pwm,0);
        analogWrite(up_pwm,0);   
        analogWrite(down_pwm,0);
       }
       else
       if(PS3.getAnalogHat(LeftHatX) == 255 && PS3.getAnalogHat(RightHatX) == 255)
       {
         digitalWrite(up_dir,LOW);
         digitalWrite(down_dir,HIGH);
         analogWrite(up_pwm,Speed);
         analogWrite(down_pwm,Speed);
       }
       else
       if(PS3.getAnalogHat(LeftHatX) == 0 && PS3.getAnalogHat(RightHatX) == 0)
       {
         digitalWrite(up_dir,HIGH);
         digitalWrite(down_dir,LOW);
         analogWrite(up_pwm,Speed);
         analogWrite(down_pwm,Speed);
       }
       else
       if(PS3.getAnalogHat(LeftHatY) == 255 && PS3.getAnalogHat(RightHatY) == 0) //
       {
         digitalWrite(up_dir,HIGH);
         digitalWrite(down_dir,HIGH);
         digitalWrite(left_dir,HIGH);
         digitalWrite(right_dir,HIGH);
         analogWrite(up_pwm,Speed);
         analogWrite(down_pwm,Speed);
         analogWrite(left_pwm,Speed);
         analogWrite(right_pwm,Speed);
       }
       else 
       if(PS3.getAnalogHat(LeftHatY) == 0 && PS3.getAnalogHat(RightHatY) == 255)
       {
         digitalWrite(up_dir,LOW);
         digitalWrite(down_dir,LOW);
         digitalWrite(left_dir,LOW);
         digitalWrite(right_dir,LOW);
         analogWrite(up_pwm,Speed);
         analogWrite(down_pwm,Speed);
         analogWrite(left_pwm,Speed);
         analogWrite(right_pwm,Speed);
       }
       else 
       if(PS3.getAnalogHat(LeftHatY) == 0 && PS3.getAnalogHat(RightHatY) == 0)
       {
         digitalWrite(left_dir,LOW);
         digitalWrite(right_dir,HIGH);
         analogWrite(left_pwm,Speed);
         analogWrite(right_pwm,Speed);
       }
       else 
       if(PS3.getAnalogHat(LeftHatY) == 255 && PS3.getAnalogHat(RightHatY) == 255)
       {
         digitalWrite(left_dir,HIGH);
         digitalWrite(right_dir,LOW);
         analogWrite(left_pwm,Speed);
         analogWrite(right_pwm,Speed);
       }
       else
       {
        
         if(PS3.getAnalogHat(LeftHatY) == 255)
         {
            digitalWrite(left_dir,HIGH);
            analogWrite(left_pwm,Speed);
         }
         else
         if(PS3.getAnalogHat(LeftHatY) == 0)
         {
            digitalWrite(left_dir,LOW);
            analogWrite(left_pwm,Speed);
         } 
         else
         if(PS3.getAnalogHat(LeftHatX) == 255)
         {
           digitalWrite(up_dir,LOW);
           digitalWrite(down_dir,HIGH);
           analogWrite(up_pwm,Speed);
           analogWrite(down_pwm,Speed);
         } 
         else
         if(PS3.getAnalogHat(LeftHatX) == 0)
         {
           digitalWrite(up_dir,HIGH);
           digitalWrite(down_dir,LOW);
           analogWrite(up_pwm,Speed);
           analogWrite(down_pwm,Speed);        
         }
         else
         if(PS3.getAnalogHat(RightHatY) == 255)
         {
           digitalWrite(right_dir,LOW);
           analogWrite(right_pwm,Speed);
         }
         else
         if(PS3.getAnalogHat(RightHatY) == 0)
         {
           digitalWrite(right_dir,HIGH);
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
