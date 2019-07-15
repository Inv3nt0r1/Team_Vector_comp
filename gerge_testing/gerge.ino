/*Program to test gerge passing manually */ 

#include <Servo.h> 
 
Servo myservo;
Servo yourservo;// create servo object to control a servo 
                
 
int pos = 0;
int pes =0;// variable to store the servo position 
char val=2; 
void setup() 
{ 
  myservo.attach(8);  // attaches the servo on pin 8 to the servo object 
 
  yourservo.attach(10);
 
  Serial.begin(9600);
} 
 
void loop() 
{
 
   if(Serial.available()>0){
   
   val=Serial.read();
   
  if(val=='1')
  {
    Serial.println(val);
  for(pos = 0; pos <= 125; pos += 1) // goes from 0 degrees to 125 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);
    if(pos<=90)
    {
      yourservo.write(pos);
    }
    delay(500);
     
    Serial.println(pos); 
  }
  
  
  } 
  
  else if(val=='0')
  {
    Serial.println(val);
  for(pos = 125; pos>=0; pos-=1)     // goes from 125 degrees to 0 degrees 
 {                                
   myservo.write(pos);
   
    if(pos<=90)
    {
      yourservo.write(pos);
    }   
   // tell servo to go to position in variable 'pos' 
    delay(500);
    Serial.println(pos); 
 }
 
 }
  
    }
   }
 
