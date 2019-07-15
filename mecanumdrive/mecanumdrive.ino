

//  Mecanum wheel drive using 2 H-Bridge( L298N ) and controlled by PS3 controller

#include <PS3USB.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

// motor1
int en1=3,in11=4,in12=5;

// motor2
int en2=6,in13=7,in14=8;

// Motor3
int en3=9,in21=10,in22=11;

// motor4
int en4=44,in23=45,in24=46;

int pwm;
USB Usb;
PS3USB PS3(&Usb); 

void setup() 
{
  pinMode(en1,OUTPUT);
  pinMode(en2,OUTPUT);
  pinMode(en3,OUTPUT);
  pinMode(en4,OUTPUT);
  pinMode(in11,OUTPUT);
  pinMode(in12,OUTPUT);
  pinMode(in13,OUTPUT);
  pinMode(in14,OUTPUT);
  pinMode(in21,OUTPUT);
  pinMode(in22,OUTPUT);
  pinMode(in23,OUTPUT);
  pinMode(in24,OUTPUT);
  Serial.begin(115200);
  
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 USB Library Started"));
}


void loop() 
{
  Usb.Task();
        
        if(PS3.getAnalogHat(LeftHatY)<117 && PS3.getAnalogHat(LeftHatX)==127)                    //GO forward
        {
            forward(); 
        }
        else if(PS3.getAnalogHat(LeftHatY)>137 && PS3.getAnalogHat(LeftHatX)==127)               //Go backward 
        {
            backward();
        }
        else if(PS3.getAnalogHat(LeftHatX)<117 && PS3.getAnalogHat(LeftHatY)==127)               //Turn Left
        {
            left();  
        }
        else if(PS3.getAnalogHat(LeftHatX)>137 && PS3.getAnalogHat(LeftHatY)==127)               //Trun Right
        {
            right();  
        }
        else if(PS3.getAnalogHat(LeftHatX)<117 && PS3.getAnalogHat(LeftHatY)<117)                //Left-forward
        {
            leftforward();  
        }
        else if(PS3.getAnalogHat(LeftHatX)>137 && PS3.getAnalogHat(LeftHatY)<117)                //Right-Forward
        {
            rightforward();  
        }
        else if(PS3.getAnalogHat(LeftHatX)<117 && PS3.getAnalogHat(LeftHatY)>135 )                //Left-Backward
        {
            leftbackward();  
        }
        else if(PS3.getAnalogHat(LeftHatX)>135 && PS3.getAnalogHat(LeftHatY)>135)                //Right-Backward
        {
            rightbackward();  
        }
        else                                                                                     // Bot at rest
        {
            steady();
        }
        if(PS3.getAnalogHat(RightHatX)<117 )                                      // For anti-clockwise Rotation
        {
            anticlockwise();  
        }
        else if(PS3.getAnalogHat(RightHatX)>135)                                // For clockwise Rotation
        {
            clockwise();  
        }
   
}


void forward()
{
      pwm=map(PS3.getAnalogHat(LeftHatY),117,0,0,50);
      digitalWrite(in11,HIGH);
      digitalWrite(in12,LOW);
      digitalWrite(in13,LOW);
      digitalWrite(in14,HIGH);
      digitalWrite(in21,HIGH);
      digitalWrite(in22,LOW);
      digitalWrite(in23,LOW);
      digitalWrite(in24,HIGH);
      analogWrite(en1,pwm);
      analogWrite(en2,pwm);
      analogWrite(en3,pwm);
      analogWrite(en4,pwm);
}
void backward()
{
      pwm=map(PS3.getAnalogHat(LeftHatY),137,255,0,50);
      digitalWrite(in11,LOW);
      digitalWrite(in12,HIGH);
      digitalWrite(in13,HIGH);
      digitalWrite(in14,LOW);
      digitalWrite(in21,LOW);
      digitalWrite(in22,HIGH);
      digitalWrite(in23,HIGH);
      digitalWrite(in24,LOW);
      analogWrite(en1,pwm);
      analogWrite(en2,pwm);
      analogWrite(en3,pwm);
      analogWrite(en4,pwm);  
}
void left()
{
      pwm=map(PS3.getAnalogHat(LeftHatX),117,0,0,50);
      digitalWrite(in11,LOW);
      digitalWrite(in12,HIGH);
      digitalWrite(in13,LOW);
      digitalWrite(in14,HIGH);
      digitalWrite(in21,HIGH);
      digitalWrite(in22,LOW);
      digitalWrite(in23,HIGH);
      digitalWrite(in24,LOW);
      analogWrite(en1,pwm);
      analogWrite(en2,pwm);
      analogWrite(en3,pwm);
      analogWrite(en4,pwm); 
}
void right()
{
      pwm=map(PS3.getAnalogHat(LeftHatX),137,255,0,50);
      digitalWrite(in11,HIGH);
      digitalWrite(in12,LOW);
      digitalWrite(in13,HIGH);
      digitalWrite(in14,LOW);
      digitalWrite(in21,LOW);
      digitalWrite(in22,HIGH);
      digitalWrite(in23,LOW);
      digitalWrite(in24,HIGH);
      analogWrite(en1,pwm);
      analogWrite(en2,pwm);
      analogWrite(en3,pwm);
      analogWrite(en4,pwm); 
}
void leftforward()
{
      pwm=map(PS3.getAnalogHat(LeftHatY),117,0,0,50);
      digitalWrite(in11,LOW);
      digitalWrite(in12,LOW);
      digitalWrite(in13,LOW);
      digitalWrite(in14,HIGH);
      digitalWrite(in21,HIGH);
      digitalWrite(in22,LOW);
      digitalWrite(in23,LOW);
      digitalWrite(in24,LOW);
      analogWrite(en1,30);
      analogWrite(en2,pwm);
      analogWrite(en3,pwm);
      analogWrite(en4,30);
}
void rightforward()
{
      pwm=map(PS3.getAnalogHat(LeftHatY),117,0,0,50);
      digitalWrite(in11,HIGH);
      digitalWrite(in12,LOW);
      digitalWrite(in13,LOW);
      digitalWrite(in14,LOW);
      digitalWrite(in21,LOW);
      digitalWrite(in22,LOW);
      digitalWrite(in23,LOW);
      digitalWrite(in24,HIGH);
      analogWrite(en1,pwm);
      analogWrite(en2,30);
      analogWrite(en3,30);
      analogWrite(en4,pwm);
}
void leftbackward()
{
      pwm=map(PS3.getAnalogHat(LeftHatY),137,255,0,50);
      digitalWrite(in11,LOW);
      digitalWrite(in12,HIGH);
      digitalWrite(in13,LOW);
      digitalWrite(in14,LOW);
      digitalWrite(in21,LOW);
      digitalWrite(in22,LOW);
      digitalWrite(in23,HIGH);
      digitalWrite(in24,LOW);
      analogWrite(en1,pwm);
      analogWrite(en2,30);
      analogWrite(en3,30);
      analogWrite(en4,pwm);
}
void rightbackward()
{
      pwm=map(PS3.getAnalogHat(LeftHatY),137,255,0,50);
      digitalWrite(in11,LOW);
      digitalWrite(in12,LOW);
      digitalWrite(in13,HIGH);
      digitalWrite(in14,LOW);
      digitalWrite(in21,LOW);
      digitalWrite(in22,HIGH);
      digitalWrite(in23,LOW);
      digitalWrite(in24,LOW);
      analogWrite(en1,30);
      analogWrite(en2,pwm);
      analogWrite(en3,pwm);
      analogWrite(en4,30);
}
void clockwise()
{
      pwm=map(PS3.getAnalogHat(RightHatX),137,255,0,50);
      digitalWrite(in11,HIGH);
      digitalWrite(in12,LOW);
      digitalWrite(in13,HIGH);
      digitalWrite(in14,LOW);
      digitalWrite(in21,HIGH);
      digitalWrite(in22,LOW);
      digitalWrite(in23,HIGH);
      digitalWrite(in24,LOW);
      analogWrite(en1,pwm);
      analogWrite(en2,pwm);
      analogWrite(en3,pwm);
      analogWrite(en4,pwm);  
}
void anticlockwise()
{
     
      pwm=map(PS3.getAnalogHat(RightHatX),117,0,0,50);
      digitalWrite(in11,LOW);
      digitalWrite(in12,HIGH);
      digitalWrite(in13,LOW);
      digitalWrite(in14,HIGH);
      digitalWrite(in21,LOW);
      digitalWrite(in22,HIGH);
      digitalWrite(in23,LOW);
      digitalWrite(in24,HIGH);
      analogWrite(en1,pwm);
      analogWrite(en2,pwm);
      analogWrite(en3,pwm);
      analogWrite(en4,pwm);
}
void steady()
{
      digitalWrite(in11,LOW);
      digitalWrite(in12,LOW);
      digitalWrite(in13,LOW);
      digitalWrite(in14,LOW);
      digitalWrite(in21,LOW);
      digitalWrite(in22,LOW);
      digitalWrite(in23,LOW);
      digitalWrite(in24,LOW);
      analogWrite(en1,0);
      analogWrite(en2,0);
      analogWrite(en3,0);
      analogWrite(en4,0);  
}
