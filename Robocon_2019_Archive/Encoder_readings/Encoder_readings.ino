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
volatile long en_left_T = 0;
volatile long en_left_B = 0;
volatile long en_right_T = 0;
volatile long en_right_B = 0;

volatile long last_left_T = 0;
volatile long last_left_B = 0;
volatile long last_right_T = 0;
volatile long last_right_B = 0;
//-----------------------------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  //-------Encoder pins------------
  pinMode(left_top_A, INPUT);
  pinMode(left_top_B, INPUT);
  pinMode(left_bottom_A, INPUT);
  pinMode(left_bottom_B, INPUT);
  pinMode(right_top_A, INPUT);
  pinMode(right_top_B, INPUT);
  pinMode(right_bottom_A, INPUT);
  pinMode(right_bottom_B, INPUT);

  digitalWrite(left_top_A, HIGH);
  digitalWrite(left_top_B, HIGH);
  digitalWrite(left_bottom_A, HIGH);
  digitalWrite(left_bottom_B, HIGH);
  digitalWrite(right_top_A, HIGH);
  digitalWrite(right_top_B, HIGH);
  digitalWrite(right_bottom_A, HIGH);
  digitalWrite(right_bottom_B, HIGH);

  attachInterrupt(digitalPinToInterrupt(left_top_A), en_left_top, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_top_B), en_left_top, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_bottom_A), en_left_bottom, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_bottom_B), en_left_bottom, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_top_A), en_right_top, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_top_B), en_right_top, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_bottom_A), en_right_bottom, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_bottom_B), en_right_bottom, CHANGE);

}

void loop() {
  // put your main code here, to run repeatedly:
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
