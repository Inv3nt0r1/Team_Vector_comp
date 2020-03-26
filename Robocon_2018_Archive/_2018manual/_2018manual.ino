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
#define lefttop_pwm 13
#define leftbottom_pwm 12
#define righttop_pwm 11
#define rightbottom_pwm 9
#define lefttop_dir 22
#define leftbottom_dir 23
#define righttop_dir 24
#define rightbottom_dir 25
//--------------------------------------------

//----------------Pneumatics------------------
#define pneu_up 26
#define pneu_down 27
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
    if (PS3.getButtonClick(PS)) 
    {
        Serial.print(F("\r\nPS"));
        PS3.disconnect();
    }
    else
    {
        if (PS3.PS3Connected || PS3.PS3NavigationConnected)
        {
            if(PS3.getButtonClick(R2)) // pneumatics up
            {
                digitalWrite(pneu_up,HIGH);
                delay(100);
                digitalWrite(pneu_up,LOW);
            }
            if(PS3.getButtonClick(L2)) // pneumatics down
            {
                digitalWrite(pneu_down,HIGH);
                delay(100);
                digitalWrite(pneu_down,LOW);
            }
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
        analogWrite(lefttop_pwm,0);
        analogWrite(leftbottom_pwm,0);
        analogWrite(rightbottom_pwm,0);   
        analogWrite(righttop_pwm,0);
       }
       else
       if(PS3.getAnalogHat(LeftHatX) == 0 && PS3.getAnalogHat(RightHatX) == 255)
       {
        analogWrite(lefttop_pwm,0);
        analogWrite(leftbottom_pwm,0);
        analogWrite(rightbottom_pwm,0);   
        analogWrite(righttop_pwm,0);
       }
       else
       if(PS3.getAnalogHat(LeftHatX) == 255 && PS3.getAnalogHat(RightHatX) == 255)
       {
           digitalWrite(lefttop_dir,LOW);
           digitalWrite(leftbottom_dir,HIGH);
           digitalWrite(righttop_dir,LOW);
           digitalWrite(rightbottom_dir,HIGH);
           analogWrite(lefttop_pwm,Speed);   
           analogWrite(leftbottom_pwm,Speed); 
           analogWrite(righttop_pwm,Speed);   
           analogWrite(rightbottom_pwm,Speed); 
       }
       else
       if(PS3.getAnalogHat(LeftHatX) == 0 && PS3.getAnalogHat(RightHatX) == 0)
       {
           digitalWrite(lefttop_dir,HIGH);
           digitalWrite(leftbottom_dir,LOW);
           digitalWrite(righttop_dir,HIGH);
           digitalWrite(rightbottom_dir,LOW);
           analogWrite(lefttop_pwm,Speed);   
           analogWrite(leftbottom_pwm,Speed);  
           analogWrite(righttop_pwm,Speed);   
           analogWrite(rightbottom_pwm,Speed);
       }
       else
       if(PS3.getAnalogHat(LeftHatY) == 255 && PS3.getAnalogHat(RightHatY) == 0) //
       {
        digitalWrite(lefttop_dir,HIGH);
        digitalWrite(leftbottom_dir,HIGH);
        digitalWrite(righttop_dir,HIGH);
        digitalWrite(rightbottom_dir,HIGH);
        analogWrite(lefttop_pwm,Speed);
        analogWrite(leftbottom_pwm,Speed);
        analogWrite(rightbottom_pwm,Speed);   
        analogWrite(righttop_pwm,Speed);
       }
       else 
       if(PS3.getAnalogHat(LeftHatY) == 0 && PS3.getAnalogHat(RightHatY) == 255)
       {
        digitalWrite(lefttop_dir,LOW);
        digitalWrite(leftbottom_dir,LOW);
        digitalWrite(righttop_dir,LOW);
        digitalWrite(rightbottom_dir,LOW);
        analogWrite(lefttop_pwm,Speed);   
        analogWrite(leftbottom_pwm,Speed);
        analogWrite(rightbottom_pwm,Speed);
        analogWrite(righttop_pwm,Speed);
       }
       else 
       if(PS3.getAnalogHat(LeftHatY) == 0 && PS3.getAnalogHat(RightHatY) == 0)
       {
        digitalWrite(lefttop_dir,LOW);
        digitalWrite(leftbottom_dir,LOW);
        digitalWrite(righttop_dir,HIGH);
        digitalWrite(rightbottom_dir,HIGH);
        analogWrite(lefttop_pwm,Speed);   
        analogWrite(leftbottom_pwm,Speed);
        analogWrite(rightbottom_pwm,Speed);
        analogWrite(righttop_pwm,Speed);
       }
       else 
       if(PS3.getAnalogHat(LeftHatY) == 255 && PS3.getAnalogHat(RightHatY) == 255)
       {
        digitalWrite(lefttop_dir,HIGH);
        digitalWrite(leftbottom_dir,HIGH);
        digitalWrite(righttop_dir,LOW);
        digitalWrite(rightbottom_dir,LOW);
        analogWrite(lefttop_pwm,Speed);   
        analogWrite(leftbottom_pwm,Speed);
        analogWrite(rightbottom_pwm,Speed);
        analogWrite(righttop_pwm,Speed);
       }
       else
       {
        
         if(PS3.getAnalogHat(LeftHatY) == 255)
         {
          digitalWrite(lefttop_dir,HIGH);
          digitalWrite(leftbottom_dir,HIGH);
          analogWrite(lefttop_pwm,Speed);   
          analogWrite(leftbottom_pwm,Speed);  
         }
         else
         if(PS3.getAnalogHat(LeftHatY) == 0)
         {
          digitalWrite(lefttop_dir,LOW);
          digitalWrite(leftbottom_dir,LOW);
          analogWrite(lefttop_pwm,Speed);   
          analogWrite(leftbottom_pwm,Speed);   
         } 
         else
         if(PS3.getAnalogHat(LeftHatX) == 255)
         {
           digitalWrite(lefttop_dir,LOW);
           digitalWrite(leftbottom_dir,HIGH);
           digitalWrite(righttop_dir,LOW);
           digitalWrite(rightbottom_dir,HIGH);
           analogWrite(lefttop_pwm,Speed);   
           analogWrite(leftbottom_pwm,Speed); 
           analogWrite(righttop_pwm,Speed);   
           analogWrite(rightbottom_pwm,Speed);  
         } 
         else
         if(PS3.getAnalogHat(LeftHatX) == 0)
         {
           digitalWrite(lefttop_dir,HIGH);
           digitalWrite(leftbottom_dir,LOW);
           digitalWrite(righttop_dir,HIGH);
           digitalWrite(rightbottom_dir,LOW);
           analogWrite(lefttop_pwm,Speed);   
           analogWrite(leftbottom_pwm,Speed);  
           analogWrite(righttop_pwm,Speed);   
           analogWrite(rightbottom_pwm,Speed);  
         }
         else
         if(PS3.getAnalogHat(RightHatY) == 255)
         {
           digitalWrite(righttop_dir,LOW);
           digitalWrite(rightbottom_dir,LOW);
           analogWrite(righttop_pwm,Speed);
           analogWrite(rightbottom_pwm,Speed);
         }
         else
         if(PS3.getAnalogHat(RightHatY) == 0)
         {
           digitalWrite(righttop_dir,HIGH);
           digitalWrite(rightbottom_dir,HIGH);
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
