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

/*
 * Wheel Arrangement:

    |      ---
    |


             |
    ---      |

*/



#include <PS3BT.h>

USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd);

//----------Motor pins------------
int const lefttop_pwm = 3; 
int const leftbottom_pwm = 7;
int const righttop_pwm = 5;
int const rightbottom_pwm = 8;
int const lefttop_dir = 2;
int const leftbottom_dir = 9;
int const righttop_dir = 4;
int const rightbottom_dir = 10;
//--------------------------------

//--------Encoder pins------------
int const left_top_A = 1;
int const left_top_B = 6;
int const left_bottom_A = 11;
int const left_bottom_B = 12;
int const right_top_A = 13;
int const right_top_B = 14;
int const right_bottom_A = 15;
int const right_bottom_B = 16;
//---------------------------------

//-----------------------Encoder Values----------------------------------
volatile long en_left_T =0;//,en_left_B,en_right_T,en_right_B;
//en_left_T = en_left_B = en_right_T = en_right_B = 0;

volatile long last_left_T=0;//,last_left_B,last_right_T,last_right_B;
//last_left_T = last_left_B = last_right_T = last_right_B = 0;
//-----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
  //------Motor Pinmodes-------------
  pinMode(lefttop_pwm, OUTPUT);
  pinMode(leftbottom_pwm, OUTPUT);
  pinMode(righttop_pwm, OUTPUT);
  pinMode(rightbottom_pwm, OUTPUT);
  pinMode(lefttop_dir, OUTPUT);
  pinMode(leftbottom_dir, OUTPUT);
  pinMode(righttop_dir, OUTPUT);
  pinMode(rightbottom_dir, OUTPUT);
  //---------------------------------

  //-------Encoder pins------------
  pinMode(left_top_A,INPUT);
  pinMode(left_top_B,INPUT);
  pinMode(left_bottom_A,INPUT);
  pinMode(left_bottom_B,INPUT);
  pinMode(right_top_A,INPUT);
  pinMode(right_top_B,INPUT);
  pinMode(right_bottom_A,INPUT);
  pinMode(right_bottom_B,INPUT);
  
  attachInterrupt(digitalPinToInterrupt(left_top_A), en_left_top,CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_top_B), en_left_top,CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_bottom_A), en_left_bottom,CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_bottom_B), en_left_bottom,CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_top_A), en_right_top,CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_top_B), en_right_top,CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_bottom_A), en_right_bottom, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_bottom_B), en_right_bottom,CHANGE);
  
  //----------------------------------
}

void loop() {
  Usb.Task();

  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {

    if(PS3.getAnalogHat(LeftHatX)==255 && PS3.getAnalogHat(RightHatX)==255) //Go right
       {
           digitalWrite(lefttop_dir,HIGH);
           digitalWrite(leftbottom_dir,LOW);
           digitalWrite(righttop_dir,HIGH);
           digitalWrite(rightbottom_dir,LOW);
           Serial.println("bot right");
           analogWrite(lefttop_pwm,0);   
           analogWrite(leftbottom_pwm,Speed); 
           analogWrite(righttop_pwm,Speed);   
           analogWrite(rightbottom_pwm,0); 
       }
       else
       if(PS3.getAnalogHat(LeftHatX)==0 && PS3.getAnalogHat(RightHatX) ==0)  //Go Left
       {
           digitalWrite(lefttop_dir,LOW);
           digitalWrite(leftbottom_dir,HIGH);
           digitalWrite(righttop_dir,LOW);
           digitalWrite(rightbottom_dir,HIGH);
           Serial.println("bot left");
           analogWrite(lefttop_pwm,0);   
           analogWrite(leftbottom_pwm,Speed);  
           analogWrite(righttop_pwm,Speed);   
           analogWrite(rightbottom_pwm,0);
       }
       else
       if(PS3.getAnalogHat(LeftHatY)==255 && PS3.getAnalogHat(RightHatY)==0)  //Spin anticlockwise
       {
        digitalWrite(lefttop_dir,LOW);
        digitalWrite(leftbottom_dir,LOW);
        digitalWrite(righttop_dir,LOW);
        digitalWrite(rightbottom_dir,LOW);
        Serial.println("bot anticlock");
        analogWrite(lefttop_pwm,Speed);
        analogWrite(leftbottom_pwm,Speed);
        analogWrite(rightbottom_pwm,Speed);   
        analogWrite(righttop_pwm,Speed);
       }
       else 
       if(PS3.getAnalogHat(LeftHatY)==0 && PS3.getAnalogHat(RightHatY)==255)   //Spin Clockwise
       {
        digitalWrite(lefttop_dir,HIGH);
        digitalWrite(leftbottom_dir,HIGH);
        digitalWrite(righttop_dir,HIGH);
        digitalWrite(rightbottom_dir,HIGH);
        Serial.println("bot clock");
        analogWrite(lefttop_pwm,Speed);   
        analogWrite(leftbottom_pwm,Speed);
        analogWrite(rightbottom_pwm,Speed);
        analogWrite(righttop_pwm,Speed);
       }
       else 
       if(PS3.getAnalogHat(LeftHatY)==255 && PS3.getAnalogHat(RightHatY)==255)  //Go forward
       {
        digitalWrite(lefttop_dir,HIGH);
        digitalWrite(leftbottom_dir,LOW);
        digitalWrite(righttop_dir,LOW);
        digitalWrite(rightbottom_dir,LOW);
        Serial.println("bot front");
        analogWrite(lefttop_pwm,Speed);   
        analogWrite(leftbottom_pwm,0);
        analogWrite(rightbottom_pwm,Speed);
        analogWrite(righttop_pwm,0);
       }
       else 
       if(PS3.getAnalogHat(LeftHatX)==0 && PS3.getAnalogHat(RightHatY)==0)    //Go Backward
       {
        digitalWrite(lefttop_dir,LOW);
        digitalWrite(leftbottom_dir,HIGH);
        digitalWrite(righttop_dir,HIGH);
        digitalWrite(rightbottom_dir,HIGH);
        Serial.println("bot back");
        analogWrite(lefttop_pwm,Speed);   
        analogWrite(leftbottom_pwm,0);
        analogWrite(rightbottom_pwm,Speed);
        analogWrite(righttop_pwm,0);
       }

    else {
        digitalWrite(lefttop_dir,LOW);
        digitalWrite(leftbottom_dir,HIGH);
        digitalWrite(righttop_dir,HIGH);
        digitalWrite(rightbottom_dir,HIGH);
        Serial.println("Wait");
        analogWrite(lefttop_pwm,0);   
        analogWrite(leftbottom_pwm,0);
        analogWrite(rightbottom_pwm,0);
        analogWrite(righttop_pwm,0);
    }

    if (PS3.PS3Connected) {

      if (PS3.getButtonClick(PS)) {
        PS3.disconnect();
      }
    }
  }
  Serial.print("LeftTop:\t");
  Serial.println(en_left_T);
  Serial.print("RightTop:\t");
  Serial.println(en_right_T);
  Serial.print("LeftBottom:\t");
  Serial.println(en_left_B);
  Serial.print("RightBottom:\t");
  Serial.println(en_right_B);
  Serial.println();
  delay(200);
}

//Routine for top left encoder
void en_left_top()
{
  int MSB = digitalRead(left_top_A); //MSB = most significant bit
  int LSB = digitalRead(left_top_B); //LSB = least significant bit
  
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (last_left_T << 2) | encoded; //adding it to the previous encoded value

  /*
    For the next part of the code:
    Remove the comments on every individual case to increase accuracy
    1X = 600 PPR
    2X = 1200 PPR
    3X = 1800 PPR
    4X = 2400 PPR
    Include all four conditions in each 'if' statement for 4X accuracy, 3 for 3X and you pretty much get the pattern.
    
    Why the numbers you ask? I have no idea. Draw the wave forms for every sensor and figure it out for yourself.
    
  */
  if(sum == 0b1101 /*|| sum == 0b0100 || sum == 0b0010 || sum == 0b1011*/) 
      en_left_T ++;
  if(sum == 0b1110 /*|| sum == 0b0111 || sum == 0b0001 || sum == 0b1000*/) 
      enc_left_T --;
 
  last_left_T = encoded; //store this value for next time
}

//Routine for bottom left sensor. Check the first defined routine for details.
void en_left_bottom()
{
  int MSB = digitalRead(left_bottom_A); //MSB = most significant bit
  int LSB = digitalRead(left_bottom_B); //LSB = least significant bit
  
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (last_left_B << 2) | encoded; //adding it to the previous encoded value
  
  if(sum == 0b1101 /*|| sum == 0b0100 || sum == 0b0010 || sum == 0b1011*/) 
      en_left_B ++;
  if(sum == 0b1110 /*|| sum == 0b0111 || sum == 0b0001 || sum == 0b1000*/) 
      enc_left_B --;
 
  last_left_B = encoded; //store this value for next time
}

//Routine for top right sensor.
void en_right_top()
{
  int MSB = digitalRead(right_top_A); //MSB = most significant bit
  int LSB = digitalRead(right_top_B); //LSB = least significant bit
  
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (last_right_T << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 /*|| sum == 0b0100 || sum == 0b0010 || sum == 0b1011*/) 
      en_right_T ++;
  if(sum == 0b1110 /*|| sum == 0b0111 || sum == 0b0001 || sum == 0b1000*/) 
      en_right_T --;
 
  last_right_T = encoded; //store this value for next time
}

//Routine for bottom right sensor.
void en_right_bottom()
{
  int MSB = digitalRead(right_bottom_A); //MSB = most significant bit
  int LSB = digitalRead(right_bottom_B); //LSB = least significant bit
  
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (last_right_B << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 /*|| sum == 0b0100 || sum == 0b0010 || sum == 0b1011*/) 
      en_right_B ++;
  if(sum == 0b1110 /*|| sum == 0b0111 || sum == 0b0001 || sum == 0b1000*/) 
      enc_right_B --;
 
  last_right_B = encoded; //store this value for next time
}
