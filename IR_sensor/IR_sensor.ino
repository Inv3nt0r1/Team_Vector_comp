int IRpin=2;
int isobstacle;

void setup()
{
      pinMode(IRpin,INPUT);

      Serial.begin(9600);

}

void loop()
{

      isobstacle=digitalRead(IRpin);

      if(isobstacle==LOW)
      {
            Serial.println("Stop");  
      }
      else
      {
            Serial.println("No obstacle detected");  
      }

}
