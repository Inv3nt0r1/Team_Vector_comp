
// Line following bot using LSA08 in serial mode  

// created on: 28/01/2019 
// Last updated on: 28/01/2019



#define rx 0                  // Defining pin o as rx
#define tx 1                  // Defining pin 1 as tx
#define pwm_left 6            // 
#define pwm_right 5
#define Right_motor 3
#define Left_motor 4
#define serialEn 2            // UART output enable is connected to pin 2 

 
void setup() 
{
     
      pinMode(pwm_left,OUTPUT);
      pinMode(pwm_right,OUTPUT);
      pinMode(Right_motor,OUTPUT);
      pinMode(Left_motor,OUTPUT);
      pinMode(serialEn,OUTPUT);
      Serial.begin(9600);
      
}

void loop() 
{
    int Position;                       

    digitalWrite(serialEn,LOW);                     // start requesting data from UART
    Position=Serial.read();

    if(Position<=25)
    {
          move_left();  
    }
    else if(Position>=45)
    {
          move_right();  
    }
    else if(Position>25 && Position<45)
    {
          move_forward();  
    }
    else
    {
          Stop();  
    }
      
}

void move_left()
{
      analogWrite(pwm_left,30);
      analogWrite(pwm_right,60);
      
}
void move_right()
{
      analogWrite(pwm_left,60);
      analogWrite(pwm_right,30);
      
}
void move_forward()
{
      analogWrite(pwm_left,60);
      analogWrite(pwm_right,60);
      
}
void Stop()
{
      analogWrite(pwm_left,0);
      analogWrite(pwm_right,0);
      
}
