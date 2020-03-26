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
   Wheel Arrangement:

   0 |      2 ---
     |


               |
   1 ---     3 |

*/



#include <PS3USB.h>

USB Usb;
/* You can create the instance of the class in two ways */
PS3USB PS3(&Usb); // This will just create the instance
//PS3USB PS3(&Usb,0x00,0x15,0x83,0x3D,0x0A,0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

//----------Motor pins------------
int const lefttop_pwm = 20;
int const leftbottom_pwm = 21;
int const righttop_pwm = 22;
int const rightbottom_pwm = 23;
const int n=4;
int dir[n] = {16,17,18,19};
/*int const lefttop_dir = 16;
int const leftbottom_dir = 17;
int const righttop_dir = 18;
int const rightbottom_dir = 19;*/
//--------------------------------

//--------Encoder pins------------
int const left_top_A = 1;
int const left_top_B = 2;
int const left_bottom_A = 3;
int const left_bottom_B = 4;
int const right_top_A = 5;
int const right_top_B = 6;
int const right_bottom_A = 7;
int const right_bottom_B = 8;
//---------------------------------

//-----------------------Encoder Values----------------------------------
volatile long en_left_T=0; 
volatile long en_left_B = 0;
volatile long en_right_T = 0;
volatile long en_right_B = 0;

volatile long last_left_T = 0;
volatile long last_left_B = 0;
volatile long last_right_T = 0;
volatile long last_right_B = 0;
//-----------------------------------------------------------------------

int Speed=25;

double Speed_l,Speed_r;
int E=600;                //Number of ticks per revolution


double Kp;
double Kd;
double Ki;
int Time = 50;
int min_pwm = 15;
int max_pwm = 40;
double setpoint[n];
double total_error[n];
double last_error[n];
unsigned long last_time[n];

void setup() 
{
  Serial.begin(115200);
  if (Usb.Init() == -1) 
  {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
  //------Motor Pinmodes-------------
  pinMode(lefttop_pwm, OUTPUT);
  pinMode(leftbottom_pwm, OUTPUT);
  pinMode(righttop_pwm, OUTPUT);
  pinMode(rightbottom_pwm, OUTPUT);
  pinMode(dir[0], OUTPUT);
  pinMode(dir[1], OUTPUT);
  pinMode(dir[2], OUTPUT);
  pinMode(dir[3], OUTPUT);
  //---------------------------------

  //-------Encoder pins------------
  pinMode(left_top_A, INPUT);
  pinMode(left_top_B, INPUT);
  pinMode(left_bottom_A, INPUT);
  pinMode(left_bottom_B, INPUT);
  pinMode(right_top_A, INPUT);
  pinMode(right_top_B, INPUT);
  pinMode(right_bottom_A, INPUT);
  pinMode(right_bottom_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(left_top_A), en_left_top, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_top_B), en_left_top, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_bottom_A), en_left_bottom, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_bottom_B), en_left_bottom, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_top_A), en_right_top, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_top_B), en_right_top, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_bottom_A), en_right_bottom, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_bottom_B), en_right_bottom, CHANGE);

  //----------------------------------
}

void loop() 
{
  Usb.Task();

  if (PS3.PS3Connected || PS3.PS3NavigationConnected) 
  {
        
    if (PS3.getAnalogHat(LeftHatX) == 255 && PS3.getAnalogHat(RightHatX) == 255) //Go right
    {
      digitalWrite(dir[0], HIGH);
      digitalWrite(dir[1], LOW);
      digitalWrite(dir[2], HIGH);
      digitalWrite(dir[3], LOW);
      Serial.println("bot right");
      analogWrite(lefttop_pwm, 0);
      analogWrite(leftbottom_pwm, Speed);
      analogWrite(righttop_pwm, Speed);
      analogWrite(rightbottom_pwm, 0);
    }
    else if (PS3.getAnalogHat(LeftHatX) == 0 && PS3.getAnalogHat(RightHatX) == 0) //Go Left
    {
      digitalWrite(dir[0], LOW);
      digitalWrite(dir[1], HIGH);
      digitalWrite(dir[2], LOW);
      digitalWrite(dir[3], HIGH);
      Serial.println("bot left");
      analogWrite(lefttop_pwm, 0);
      analogWrite(leftbottom_pwm, Speed);
      analogWrite(righttop_pwm, Speed);
      analogWrite(rightbottom_pwm, 0);
    }
    else if (PS3.getAnalogHat(LeftHatY) == 255 && PS3.getAnalogHat(RightHatY) == 0) //Spin anticlockwise
    {
      digitalWrite(dir[0], LOW);
      digitalWrite(dir[1], LOW);
      digitalWrite(dir[2], LOW);
      digitalWrite(dir[3], LOW);
      Serial.println("bot anticlock");
      analogWrite(lefttop_pwm, Speed);
      analogWrite(leftbottom_pwm, Speed);
      analogWrite(rightbottom_pwm, Speed);
      analogWrite(righttop_pwm, Speed);
    }
    else if (PS3.getAnalogHat(LeftHatY) == 0 && PS3.getAnalogHat(RightHatY) == 255) //Spin Clockwise
    {
      digitalWrite(dir[0], HIGH);
      digitalWrite(dir[1], HIGH);
      digitalWrite(dir[2], HIGH);
      digitalWrite(dir[3], HIGH);
      Serial.println("bot clock");
      analogWrite(lefttop_pwm, Speed);
      analogWrite(leftbottom_pwm, Speed);
      analogWrite(rightbottom_pwm, Speed);
      analogWrite(righttop_pwm, Speed);
    }
    else if (PS3.getAnalogHat(LeftHatY) == 0 && PS3.getAnalogHat(RightHatY) == 0) //Go forward
    {
      digitalWrite(dir[0], HIGH);
      digitalWrite(dir[1], LOW);
      digitalWrite(dir[2], LOW);
      digitalWrite(dir[3], LOW);
      Serial.println("bot front");
      if((setpoint[0] - en_left_T < 5*E && en_right_B - setpoint[3] < 5*E) || en_left_T - setpoint[0] < 5*E && setpoint[3] - en_right_B < 5*E)
      {
        Speed_l = PID(en_left_T, 0);
        Speed_r = PID(en_right_B, 3);
      } 
      else
      {
        Speed_l = Speed;
        Speed_r = Speed;
      }
      analogWrite(lefttop_pwm, Speed_l);
      analogWrite(leftbottom_pwm, 0);
      analogWrite(rightbottom_pwm, Speed_r);
      analogWrite(righttop_pwm, 0);
    }
    else if (PS3.getAnalogHat(LeftHatY) == 255 && PS3.getAnalogHat(RightHatY) == 255) //Go Backward
    {
      digitalWrite(dir[0], LOW);
      digitalWrite(dir[1], HIGH);
      digitalWrite(dir[2], HIGH);
      digitalWrite(dir[3], HIGH);
      Serial.println("bot back");
      analogWrite(lefttop_pwm, Speed);
      analogWrite(leftbottom_pwm, 0);
      analogWrite(rightbottom_pwm, Speed);
      analogWrite(righttop_pwm, 0);
    }

    else {
      digitalWrite(dir[0], LOW);
      digitalWrite(dir[1], HIGH);
      digitalWrite(dir[2], HIGH);
      digitalWrite(dir[3], HIGH);
      Serial.println("Wait");
      analogWrite(lefttop_pwm, 0);
      analogWrite(leftbottom_pwm, 0);
      analogWrite(rightbottom_pwm, 0);
      analogWrite(righttop_pwm, 0);
    }

    if (PS3.PS3Connected) 
    {

      if (PS3.getButtonClick(PS)) 
      {
        //PS3.disconnect();
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

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
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
  if (sum == 0b1101 /*|| sum == 0b0100 || sum == 0b0010 || sum == 0b1011*/)
    en_left_T ++;
  if (sum == 0b1110 /*|| sum == 0b0111 || sum == 0b0001 || sum == 0b1000*/)
    en_left_T --;

  last_left_T = encoded; //store this value for next time
}

//Routine for bottom left sensor. Check the first defined routine for details.
void en_left_bottom()
{
  int MSB = digitalRead(left_bottom_A); //MSB = most significant bit
  int LSB = digitalRead(left_bottom_B); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (last_left_B << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 /*|| sum == 0b0100 || sum == 0b0010 || sum == 0b1011*/)
    en_left_B ++;
  if (sum == 0b1110 /*|| sum == 0b0111 || sum == 0b0001 || sum == 0b1000*/)
    en_left_B --;

  last_left_B = encoded; //store this value for next time
}

//Routine for top right sensor.
void en_right_top()
{
  int MSB = digitalRead(right_top_A); //MSB = most significant bit
  int LSB = digitalRead(right_top_B); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (last_right_T << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 /*|| sum == 0b0100 || sum == 0b0010 || sum == 0b1011*/)
    en_right_T ++;
  if (sum == 0b1110 /*|| sum == 0b0111 || sum == 0b0001 || sum == 0b1000*/)
    en_right_T --;

  last_right_T = encoded; //store this value for next time
}

//Routine for bottom right sensor.
void en_right_bottom()
{
  int MSB = digitalRead(right_bottom_A); //MSB = most significant bit
  int LSB = digitalRead(right_bottom_B); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (last_right_B << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 /*|| sum == 0b0100 || sum == 0b0010 || sum == 0b1011*/)
    en_right_B ++;
  if (sum == 0b1110 /*|| sum == 0b0111 || sum == 0b0001 || sum == 0b1000*/)
    en_right_B --;

  last_right_B = encoded; //store this value for next time
}


double PID(double encoder_value, int motor_no)
{
  unsigned long current_time = millis();
  unsigned long delta_time = current_time - last_time[motor_no];
  double pwm_signal = Speed;
  
  if(delta_time >= Time)
  {
    double error;
    if(encoder_value < setpoint[motor_no])
    {
      error = setpoint[motor_no] - encoder_value;
      digitalWrite(dir[motor_no],HIGH);
    }
    else
    {
      error = encoder_value - setpoint[motor_no];
      digitalWrite(dir[motor_no],LOW);
    }
    total_error[motor_no] += error;
    if(total_error[motor_no] >= max_pwm)
      total_error[motor_no] = max_pwm;
    if(total_error[motor_no] <= min_pwm)
      total_error[motor_no] = min_pwm;

    double delta_error = error - last_error[motor_no];

    pwm_signal = Kp*error + (Ki*Time)*total_error[motor_no] + (Kd/Time)*delta_error;

    if(pwm_signal >= max_pwm)
      pwm_signal = max_pwm;
    if(pwm_signal <= min_pwm)
      pwm_signal = min_pwm;
    last_error[motor_no] = error;
    last_time[motor_no] = current_time;
  }
  return pwm_signal;
}
