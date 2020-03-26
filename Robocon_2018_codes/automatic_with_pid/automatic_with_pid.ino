//hatY = 0 -> UPside
//hatY = 255 -> DOWNside
//hatX = 0 -> LEFTside
//hatX = 255 -> RIGHTside
//dir = HIGH -> Clockwise
//dir = LOW -> AntiClockwise
//arr = next-hop direction
//directions : 1 = left, 2 = right, 3 = up, 4 = down, 0 = none
//pneumatic position : tz1 , tz3 , tz2

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

//----------------Pneumatics------------------
#define pneu_tz1_up 13
#define pneu_tz1_down 14
#define pneu_tz2_up 15
#define pneu_tz2_down 16
#define pneu_tz3_up 17
#define pneu_tz3_down 18
//--------------------------------------------

//----------------LSA-08----------------------
#define rx1 0
#define tx1 1
#define serialEn1 6
#define rx2 9
#define tx2 10
#define serialEn2 7
//--------------------------------------------

// access element as [source][Destination][0 for next-hop/ 1 for direction]
int arr[7][7][2] = 
{
  {
    {0,0},
    {1,2},
    {1,2},
    {1,2},
    {1,2},
    {1,2},
    {1,2}
  },
  {
    {0,1},
    {1,0},
    {2,4},
    {2,4},
    {2,4},
    {2,4},
    {2,4}
  },
  {
    {1,3},
    {1,3},
    {2,0},
    {3,2},
    {4,4},
    {4,4},
    {4,4}
  },
  {
    {2,1},
    {2,1},
    {2,1},
    {3,0},
    {2,1},
    {2,1},
    {2,1}
  },
  {
    {2,3},
    {2,3},
    {2,3},
    {2,3},
    {4,0},
    {5,2},
    {5,2}
  },
  {
    {4,1},
    {4,1},
    {4,1},
    {4,1},
    {4,1},
    {5,0},
    {6,2}
  },
  {
    {5,1},
    {5,1},
    {5,1},
    {5,1},
    {5,1},
    {5,1},
    {6,0}
  }
};

int Source = 0, Destination = 2, NextHop, Direction, JunctionFlag = 0, Speed = 35, LeftSpeed = 35, RightSpeed = 35, kp1 = 0, kd1 = 0, kp2 = 0, kd2 = 0, SetPoint = 35, MaxSpeed = 40, lastError1 = 0, lastError2 = 0;

void setup() 
{

  Serial.begin(115200);
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
  
  //--------Initialize pneumatics--------
  pinMode(pneu_tz1_up,OUTPUT);
  pinMode(pneu_tz1_down,OUTPUT);
  pinMode(pneu_tz2_up,OUTPUT);
  pinMode(pneu_tz2_down,OUTPUT);
  pinMode(pneu_tz3_up,OUTPUT);
  pinMode(pneu_tz3_down,OUTPUT);
  //-------------------------------------

  //--------Initialize LSA-08------------
  pinMode(serialEn1,OUTPUT);
  pinMode(serialEn2,OUTPUT);
  digitalWrite(serialEn1,HIGH);
  digitalWrite(serialEn2,HIGH);
  //-------------------------------------
  Serial.println("Initialized");

  
  Pause();
  Move();
} // end of setup

void loop() 
{
  Destination = Wait();
  Serial.println("Initialized");
  if(Source != Destination)
  {
    Move();
    Throw();
    Adjust();
    if(Source == 3)
      Destination = 2;
    else if(Source == 5 || Source == 6)
      Destination = 4;
    Move();
  }
} // end of loop

void Move() // this function will move bot from source to Destination
{
  Serial.println("Moving :)");
  while(Source!=Destination)
  {
    Serial.println("Source not equal to destination");
    if(isJunction() && JunctionFlag>30) 
    {
      Pause();
      Source = NextHop;
      JunctionFlag = 0;
    }
    
    if(!isJunction())
      JunctionFlag++; 
      
    NextHop = arr[Source][Destination][0];
    Direction = arr[Source][Destination][1];

    switch(Direction)
    {
      case 0 : Pause();
               break;
      case 1 : lastError1 = PID1(serialEn1, kp1, kd1, lastError1);
               Left();
               break;
      case 2 : lastError1 = PID1(serialEn1, kp1, kd1, lastError1);
               Right();
               break;
      case 3 : lastError2 = PID1(serialEn2, kp2, kd2, lastError2);
               Up();
               break;
      case 4 : lastError2 = PID1(serialEn2, kp2, kd2, lastError2);
               Down();
               break;
    }
  }
} // end of move

void Adjust() // adjusts the robot after jerk due to throw
{
  Serial.println("adjusts the robot after jerk due to throw");
  while(1)
  {
    if(isJunction())
    {
      Pause();
      break;
    }
    else
    {
      lastError2 = PID1(serialEn2, kp2, kd2, lastError2);
      Down();
    }
  }
}

int Wait() // this function will wait until shuttlecock is loaded
{
  // return new Destination based on which arm detects the shuttlecock
} // end of wait

void Throw() // this function will throw the shuttlecock
{
  // throw shuttlecock from arm based on source
  Serial.println("throw shuttlecock from arm based on source");
  Serial.println("throw shuttlecock from arm based on source");
  if(Source == 3)
  {
    Serial.println("Source is 3");
    digitalWrite(pneu_tz1_up,HIGH);
    delay(1000);
    digitalWrite(pneu_tz1_up,LOW);
    delay(1000);
    digitalWrite(pneu_tz1_down,HIGH);
    delay(1000);
    digitalWrite(pneu_tz1_down,LOW);
  }
  else if(Source == 5)
  {
    Serial.println("Source is 5");
    digitalWrite(pneu_tz2_up,HIGH);
    delay(1000);
    digitalWrite(pneu_tz2_up,LOW);
    delay(1000);
    digitalWrite(pneu_tz2_down,HIGH);
    delay(1000);
    digitalWrite(pneu_tz2_down,LOW);
  }
  else if(Source == 6)
  {
    Serial.println("Source is 6");
    digitalWrite(pneu_tz3_up,HIGH);
    delay(1000);
    digitalWrite(pneu_tz3_up,LOW);
    delay(1000);
    digitalWrite(pneu_tz3_down,HIGH);
    delay(1000);
    digitalWrite(pneu_tz3_down,LOW);
  }
} // end of throw

bool isJunction() // returns 1 if junction detected
{
  Serial.println("Junction aala");
  digitalWrite(serialEn1,LOW);
  while(Serial.available() <= 0);
  int positionVal1 = Serial.read();
  digitalWrite(serialEn2,HIGH);
  if(positionVal1 != SetPoint)
    return 0;
  
  digitalWrite(serialEn2,LOW);
  while(Serial.available() <= 0);
  int positionVal2 = Serial.read();
  digitalWrite(serialEn2,HIGH);
  if(positionVal2 != SetPoint)
    return 0;

  return 1;

} // end of isJunction

void Pause()
{
  Serial.println("Thambla aahe");
  analogWrite(left_pwm,0);
  analogWrite(right_pwm,0);
  analogWrite(up_pwm,0);   
  analogWrite(down_pwm,0);
} // end of pause

void Left()
{
  Serial.println("Left");
  digitalWrite(up_dir,HIGH);
  digitalWrite(down_dir,LOW);
  analogWrite(up_pwm,LeftSpeed);
  analogWrite(down_pwm,RightSpeed);
} // end of left

void Right()
{
  Serial.println("Right");
  digitalWrite(up_dir,LOW);
  digitalWrite(down_dir,HIGH);
  analogWrite(up_pwm,LeftSpeed);
  analogWrite(down_pwm,RightSpeed);
} // end of Right

void Up()
{
  Serial.println("Up");
  digitalWrite(left_dir,LOW);
  digitalWrite(right_dir,HIGH);
  analogWrite(left_pwm,LeftSpeed);
  analogWrite(right_pwm,RightSpeed);
} // end of Up

void Down()
{
  Serial.println("Down");
  digitalWrite(left_dir,HIGH);
  digitalWrite(right_dir,LOW);
  analogWrite(left_pwm,LeftSpeed);
  analogWrite(right_pwm,RightSpeed);
} // end of Down

int PID1(int serialEn, int kp, int kd, int lastError)
{
  Serial.println("PID");
  digitalWrite(serialEn,LOW);
  while(Serial.available() <= 0);
  int positionVal = Serial.read();
  digitalWrite(serialEn,HIGH);

  if(positionVal == 255) 
  {
    Pause();
  }
  else 
  {
    int error = positionVal - SetPoint;
    int motorSpeed = kp * error + kd * (error - lastError);
    lastError = error;

    RightSpeed = Speed - motorSpeed;
    LeftSpeed = Speed + motorSpeed;

    if(RightSpeed > MaxSpeed) RightSpeed = MaxSpeed;
    if(LeftSpeed > MaxSpeed) LeftSpeed = MaxSpeed;

    if(RightSpeed < 0) RightSpeed = 0;
    if(LeftSpeed < 0) LeftSpeed = 0;

    return lastError;
    
  Serial.print("Right Speed is ");
  Serial.println(RightSpeed);
  Serial.print("Left Speed is ");
  Serial.println(LeftSpeed);
  }
} // end of PID1
