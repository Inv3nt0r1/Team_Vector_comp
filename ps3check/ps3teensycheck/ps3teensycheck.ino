

// Code to check PS3 controller with teensy as well as arduino

#include <PS3USB.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>


int pwm;
USB Usb;
PS3USB PS3(&Usb); 

void setup() 
{
  Serial.begin(9600);
  
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
        
        Serial.print("LeftHat X: ");
        Serial.print(PS3.getAnalogHat(LeftHatX));
        Serial.print("\t LeftHat Y: ");
        Serial.println(PS3.getAnalogHat(LeftHatY));
      
        Serial.print("RighttHat X: ");
        Serial.print(PS3.getAnalogHat(RightHatX));
        Serial.print("\t RighttHat Y: ");
        Serial.println(PS3.getAnalogHat(RightHatY));
    
        Serial.print("R2: ");
        Serial.print(PS3.getAnalogButton(R2));
        Serial.print("L2: ");
        Serial.println(PS3.getAnalogButton(L2));
    
        if(PS3.getButtonPress(TRIANGLE))
          Serial.println("Trignale pressed!");
        if(PS3.getButtonPress(SQUARE))
          Serial.println("Square Pressed!");
        if(PS3.getButtonPress(CIRCLE))
          Serial.println("Circle Pressed!");
        if(PS3.getButtonPress(CROSS))
          Serial.println("Cross Pressed!");
        delay(20);
           
}
