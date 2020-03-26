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

//----------------LIMIT SWITCH----------------
#define limit_tz1 11
#define limit_tz2 12
#define limit_tz3 26
//--------------------------------------------

// access element as [source][Destination][0 for next-hop/ 1 for direction]
int arr[13][13][2] = 
{
  {
    {0,0},
    {1,2},
    {1,2},
    {1,2},
    {1,2},
    {1,2},
    {1,2},
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
    {2,4},
    {2,4},
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
    {7,2},
    {4,4},
    {4,4},
    {4,4},
    {7,2},
    {7,2},
    {4,4},
    {4,4},
    {4,4},
    {4,4}
  },
  {
    {7,1},
    {7,1},
    {7,1},
    {3,0},
    {7,1},
    {7,1},
    {7,1},
    {7,1},
    {8,2},
    {7,1},
    {7,1},
    {7,1},
    {7,1}
  },
  {
    {2,3},
    {2,3},
    {2,3},
    {2,3},
    {4,0},
    {9,2},
    {9,2},
    {4,3},
    {4,3},
    {9,2},
    {9,2},
    {9,2},
    {9,2}
  },
  {
    {9,1},
    {9,1},
    {9,1},
    {9,1},
    {9,1},
    {5,0},
    {10,2},
    {9,1},
    {9,1},
    {9,1},
    {10,2},
    {10,2},
    {10,2}
  },
  {
    {11,1},
    {11,1},
    {11,1},
    {11,1},
    {11,1},
    {11,1},
    {6,0},
    {11,1},
    {11,1},
    {11,1},
    {11,1},
    {11,1},
    {12,2}
  },
  {
    {2,1},
    {2,1},
    {2,1},
    {3,2},
    {2,1},
    {2,1},
    {2,1},
    {7,0},
    {3,2},
    {2,1},
    {2,1},
    {2,1},
    {2,1}
  },
  {
    {3,1},
    {3,1},
    {3,1},
    {3,1},
    {3,1},
    {3,1},
    {3,1},
    {3,1},
    {8,0},
    {3,1},
    {3,1},
    {3,1},
    {3,1}
  },
  {
    {4,1},
    {4,1},
    {4,1},
    {4,1},
    {4,1},
    {5,2},
    {5,2},
    {4,1},
    {4,1},
    {9,0},
    {5,2},
    {5,2},
    {5,2}
  },
  {
    {5,1},
    {5,1},
    {5,1},
    {5,1},
    {5,1},
    {5,1},
    {11,2},
    {5,1},
    {5,1},
    {5,1},
    {10,0},
    {11,2},
    {11,2}
  },
  {
    {10,1},
    {10,1},
    {10,1},
    {10,1},
    {10,1},
    {10,1},
    {6,2},
    {10,1},
    {10,1},
    {10,1},
    {10,1},
    {11,0},
    {6,2}
  },
  {
    {6,1},
    {6,1},
    {6,1},
    {6,1},
    {6,1},
    {6,1},
    {6,1},
    {6,1},
    {6,1},
    {6,1},
    {6,1},
    {6,1},
    {12,0}
  }
};

int Source = 0, Destination = 1, NextHop, Direction, JunctionFlag = 0, Speed = 25, LeftSpeed = 25, RightSpeed = 25, SetPoint = 35, MaxSpeed = 30, lastError1 = 0, lastError2 = 0, LastDirection = 0, Rotate180 = 0, Rotate360 = 0;
float kp1 = 1.2, kd1 = 1.2*50, ki1=0, kp2 = 1.2, kd2 = 1.2*50,integral=0;

void setup() 
{
  delay(10000);
  
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

  //--------Initialize LIMIT SWITCH------------
  pinMode(limit_tz1,INPUT);
  pinMode(limit_tz2,INPUT);
  pinMode(limit_tz3,INPUT);
  //-------------------------------------
  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  
  Pause();  
  Move();
  _init();
  Destination = 2;
  Move();
} // end of setup

void loop() 
{
  if((Source == 2 || Source == 4) && Rotate180 == 1)
  {
    Rotate_180();
    Rotate180 = 0;
  }
  Destination = Wait();
  if(Source != Destination)
  {
    Move();
    Rotate_180();
    Rotate360 = 0;
    delay(300);
    //Throw();
    //Adjust();
    //_init();
    Destination = 4;
    Move();
  }
} // end of loop

void _init()
{
  Speed = 100;
  LeftSpeed = Speed;
  RightSpeed = Speed;
  MaxSpeed = 150;
}

void Move() // this function will move bot from source to Destination
{
  while(Source!=Destination)
  {
    if(isJunction() && JunctionFlag>15) 
    {
      if(NextHop != 7 && NextHop != 8 && NextHop != 9 && NextHop != 10 && NextHop != 11 && NextHop != 12)  
      {
        //Brake();
        Pause();
        delay(200);
        Rebound();
        Pause();
        _init();
        delay(200);
      }
      Source = NextHop;
      JunctionFlag = 0;
      digitalWrite(13,HIGH);
    }
    
    if(!isJunction())
    {
      JunctionFlag++; 
      digitalWrite(13,LOW);
    }
      
    NextHop = arr[Source][Destination][0];
    Direction = arr[Source][Destination][1];

    if(Rotate180 == 0 && Rotate360 == 1 && Direction != 0)
    {
      Direction = (Direction%2 == 0) ? (Direction - 1) : (Direction + 1);
    }
    
    if(Direction != LastDirection)
      _init();
    LastDirection = Direction;
    
    switch(Direction)
    {
      case 0 : Pause();
               break;
      case 1 : PID1();
               Left();
               break;
      case 2 : PID1();
               Right();
               break;
      case 3 : PID2();
               Up();
               break;
      case 4 : PID2();
               Down();
               break;
    }
  }
  if(Source == 2 || Source == 4)
  {
    Rotate180 = 1;
    Rotate360 = 1;
  }
} // end of move

void Rebound()
{
  LeftSpeed = 40;
  RightSpeed = 40;
  Speed = 40;
  MaxSpeed = 50;
  
  while(!isJunction())
  {
    switch(Direction)
    {
      case 0 : Pause();
               break;
      case 1 : PID1();
               Right();
               break;
      case 2 : PID1();
               Left();
               break;
      case 3 : PID2();
               Down();
               break;
      case 4 : PID2();
               Up();
               break;
    }
  }
  Pause();
  
  delay(200);
  LeftSpeed = 25;
  RightSpeed = 25;
  
  if(Direction == 1 || Direction == 2)
  {
    while(1)
    {
      digitalWrite(serialEn2,LOW);
      while(Serial2.available() <= 0);
      int positionVal2 = Serial2.read();
      digitalWrite(serialEn2,HIGH);

      if(positionVal2 > 35)
        Left();
      else
      if(positionVal2 < 35)
        Right();
      else
        break;
    }
  }
  else
  if(Direction == 3 || Direction == 4)
  {
    while(1)
    {
      digitalWrite(serialEn1,LOW);
      while(Serial1.available() <= 0);
      int positionVal1 = Serial1.read();
      digitalWrite(serialEn1,HIGH);

      if(positionVal1 > 35)
        Up();
      else
      if(positionVal1 < 35)
        Down();
      else
        break;
    }
  }
  Pause();
}

void Brake()
{
  switch(Direction)
  {
    case 0 : Pause();
             break;
    case 1 : PID1();
             Right();
             break;
    case 2 : PID1();
             Left();
             break;
    case 3 : PID2();
             Down();
             break;
    case 4 : PID2();
             Up();
             break;
  }
  delay(300);
}

int Wait() // this function will wait until shuttlecock is loaded & return new Destination based on which arm detects the shuttlecock
{
  Pause();
  if(!digitalRead(limit_tz1))
    return 3;
  else 
  if(!digitalRead(limit_tz2))
    return 5;
  else
  if(!digitalRead(limit_tz3))
    return 6;
  else 
    return Source;
} // end of wait

bool isJunction() // returns 1 if junction detected
{
  digitalWrite(serialEn1,LOW);
  while(Serial1.available() <= 0);
  int positionVal1 = Serial1.read();
  digitalWrite(serialEn1,HIGH);
  
  digitalWrite(serialEn2,LOW);
  while(Serial2.available() <= 0);
  int positionVal2 = Serial2.read();
  digitalWrite(serialEn2,HIGH);
  
  if((positionVal1 >= 20 && positionVal1 <=50) && (positionVal2 >= 20 && positionVal2 <=50))
    return 1;
  else
    return 0;
} // end of isJunction

void Pause()
{
  analogWrite(left_pwm,0);
  analogWrite(right_pwm,0);
  analogWrite(up_pwm,0);   
  analogWrite(down_pwm,0);
} // end of pause

void Left()
{
  digitalWrite(up_dir,HIGH);
  digitalWrite(down_dir,LOW);
  analogWrite(up_pwm,LeftSpeed);
  analogWrite(down_pwm,RightSpeed);
} // end of left

void Right()
{
  digitalWrite(up_dir,LOW);
  digitalWrite(down_dir,HIGH);
  analogWrite(up_pwm,LeftSpeed);
  analogWrite(down_pwm,RightSpeed);
} // end of Right

void Up()
{
  digitalWrite(left_dir,LOW);
  digitalWrite(right_dir,HIGH);
  analogWrite(left_pwm,LeftSpeed);
  analogWrite(right_pwm,RightSpeed);
} // end of Up

void Down()
{
  digitalWrite(left_dir,HIGH);
  digitalWrite(right_dir,LOW);
  analogWrite(left_pwm,LeftSpeed);
  analogWrite(right_pwm,RightSpeed);
} // end of Down

void PID1()
{
  int positionVal = 255;
  
  digitalWrite(serialEn1,LOW);
  while(Serial1.available() <= 0);
  positionVal = Serial1.read();
  digitalWrite(serialEn1,HIGH);
  Serial.println("Array 1");
  Serial.println(positionVal);

  if(positionVal == 255) 
  {
    Pause();
  }
  else 
  {
    int error = positionVal - SetPoint;
    integral += error; 
    int motorSpeed = kp1 * error + kd1 * (error - lastError1)+ ki1 * integral;
    lastError1 = error;

    LeftSpeed = Speed - motorSpeed;
    RightSpeed = Speed + motorSpeed;

    if(RightSpeed > MaxSpeed) RightSpeed = MaxSpeed;
    if(LeftSpeed > MaxSpeed) LeftSpeed = MaxSpeed;

    if(RightSpeed < 0) RightSpeed = 0;
    if(LeftSpeed < 0) LeftSpeed = 0;
  }
} // end of PID1

void PID2()
{
  int positionVal = 255;
  
  digitalWrite(serialEn2,LOW);
  while(Serial2.available() <= 0);
  positionVal = Serial2.read();
  digitalWrite(serialEn2,HIGH);
  Serial.println("Array 2");
  Serial.println(positionVal);

  if(positionVal == 255) 
  {
    Pause();
  }
  else 
  {
    int error = positionVal - SetPoint;
    int motorSpeed = kp2 * error + kd2 * (error - lastError2);
    lastError2 = error;

    LeftSpeed = Speed - motorSpeed;
    RightSpeed = Speed + motorSpeed;

    if(RightSpeed > MaxSpeed) RightSpeed = MaxSpeed;
    if(LeftSpeed > MaxSpeed) LeftSpeed = MaxSpeed;

    if(RightSpeed < 0) RightSpeed = 0;
    if(LeftSpeed < 0) LeftSpeed = 0;
  }
} // end of PID1

void Rotate_180()
{
  int JuncCount = 0;
  int RotateFlag = 0;
  digitalWrite(up_dir,LOW);
  digitalWrite(down_dir,LOW);
  digitalWrite(left_dir,LOW);
  digitalWrite(right_dir,LOW);
  
  while(JuncCount < 2)
  {    
    if(isJunction() && RotateFlag > 20)
    {
      JuncCount++;
      RotateFlag = 0;
    }
    else if(!isJunction())
      RotateFlag++;
    
    analogWrite(up_pwm,25);
    analogWrite(down_pwm,25);
    analogWrite(right_pwm,25);
    analogWrite(left_pwm,25);
  }
  Pause();
  delay(200);
  while(1)
  {
    digitalWrite(serialEn1,LOW);
    while(Serial1.available() <= 0);
    int positionVal1 = Serial1.read();
    digitalWrite(serialEn1,HIGH);

    if(positionVal1 < 35)
    {
      digitalWrite(up_dir,HIGH);
      digitalWrite(down_dir,HIGH);
      digitalWrite(left_dir,HIGH);
      digitalWrite(right_dir,HIGH);
    }
    else
    if(positionVal1 > 35)
    {
      digitalWrite(up_dir,LOW);
      digitalWrite(down_dir,LOW);
      digitalWrite(left_dir,LOW);
      digitalWrite(right_dir,LOW);
    }
    else
      break;
    
    analogWrite(up_pwm,15);
    analogWrite(down_pwm,15);
    analogWrite(right_pwm,15);
    analogWrite(left_pwm,15);
  }
  Pause();
}

void Adjust() // adjusts the robot after jerk due to throw
{
  LastDirection = 3;
  Rebound();
}

void Throw() // this function will throw the shuttlecock based on source
{
  if(Source == 3)
  {
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
    digitalWrite(pneu_tz3_up,HIGH);
    delay(1000);
    digitalWrite(pneu_tz3_up,LOW);
    delay(1000);
    digitalWrite(pneu_tz3_down,HIGH);
    delay(1000);
    digitalWrite(pneu_tz3_down,LOW);
  }
} // end of throw
