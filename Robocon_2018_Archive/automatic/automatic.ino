//hatY = 0 -> UPside
//hatY = 255 -> DOWNside
//hatX = 0 -> LEFTside
//hatX = 255 -> RIGHTside
//dir = HIGH -> Clockwise
//dir = LOW -> AntiClockwise
//arr = next-hop direction
//directions : 1 = left, 2 = right, 3 = up, 4 = down, 0 = none

//----------------Locomotion-------------------
#define left_pwm 2
#define right_pwm 3
#define up_pwm 4
#define down_pwm 5
#define left_dir 24
#define right_dir 25
#define up_dir 26
#define down_dir 27
//--------------------------------------------

//----------------Pneumatics------------------
#define pneu_tz1_up 13
#define pneu_tz1_down 14
#define pneu_tz2_up 15
#define pneu_tz2_down 16
#define pneu_tz3_up 17
#define pneu_tz3_down 18
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

int Source = 0, Destination = 2, NextHop, Direction, JunctionFlag = 0, Speed = 35;

void setup() 
{
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
  
  Serial.begin(9600);
  Move();
} // end of setup

void loop() 
{
  Destination = Wait();
  if(Source != Destination)
  {
    Move();
    Throw();
    if(Source == 3)
      Destination = 2;
    else if(Source == 5 || Source == 6)
      Destination = 4;
    Move();
  }
} // end of loop

void Move() // this function will move bot from source to Destination
{
  while(Source!=Destination)
  {
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
      case 1 : Left();
               break;
      case 2 : Right();
               break;
      case 3 : Up();
               break;
      case 4 : Down();
               break;
    }
  }
} // end of move

int Wait() // this function will wait until shuttlecock is loaded
{
  // return new Destination based on which arm detects the shuttlecock
} // end of wait

void Throw() // this function will throw the shuttlecock
{
  // throw shuttlecock from arm based on source
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

bool isJunction() // returns 1 if junction detected
{
  
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
  analogWrite(up_pwm,Speed);
  analogWrite(down_pwm,Speed);
} // end of left

void Right()
{
  digitalWrite(up_dir,LOW);
  digitalWrite(down_dir,HIGH);
  analogWrite(up_pwm,Speed);
  analogWrite(down_pwm,Speed);
} // end of Right

void Up()
{
  digitalWrite(left_dir,LOW);
  digitalWrite(right_dir,HIGH);
  analogWrite(left_pwm,Speed);
  analogWrite(right_pwm,Speed);
} // end of Up

void Down()
{
  digitalWrite(left_dir,HIGH);
  digitalWrite(right_dir,LOW);
  analogWrite(left_pwm,Speed);
  analogWrite(right_pwm,Speed);
} // end of Down
