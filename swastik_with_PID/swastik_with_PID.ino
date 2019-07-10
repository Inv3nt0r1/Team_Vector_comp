//----------Motor pins------------
int const lefttop_pwm = 20;
int const leftbottom_pwm = 21;
int const righttop_pwm = 22;
int const rightbottom_pwm = 23;
const int n = 4;
int dir[n] = {16, 17, 18, 19};
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
volatile long en_left_T = 0;
volatile long en_left_B = 0;
volatile long en_right_T = 0;
volatile long en_right_B = 0;

volatile long last_left_T = 0;
volatile long last_left_B = 0;
volatile long last_right_T = 0;
volatile long last_right_B = 0;
//-----------------------------------------------------------------------

int Speed = 25;

double Speed_l, Speed_r;
int E = 600;              //Number of ticks per revolution


double Kp;
double Kd;
double Ki;
int Time = 50;
int min_pwm = 15;
int max_pwm = 40;
double Setpoint[n] = {6000, 6000, 6000, 6000};
double total_error[n];
double last_error[n];
unsigned long last_time[n];

void setup()
{
  Serial.begin(9600);
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
  Serial.println("Waiting for 10 seconds before loop starts!");
  delay(10000);
}
void loop()
{
  while ((Setpoint[0] == 6000 && Setpoint[3] == 6000) || (Setpoint[0] == 0 && Setpoint[3] == 0))
  {
    Speed_l = PID(en_left_T, 0);
    Speed_r = PID(en_right_B, 3);
    analogWrite(lefttop_pwm, Speed_l);
    analogWrite(rightbottom_pwm, Speed_r);
    if (en_left_T > 6000 || en_right_B > 6000)
    {
      Setpoint[0] = Setpoint[3] = 0;
      break;
    }
    if (en_left_T < 0 || en_right_B < 0)
    {
      Setpoint[0] = Setpoint[3] = 0;
      break;
    }
  }
  while ((Setpoint[1] == 6000 && Setpoint[2] == 6000) || (Setpoint[1] == 0 && Setpoint[2] == 0))
  {
    Speed_l = PID(en_left_B, 1);
    Speed_r = PID(en_right_T, 2);
    analogWrite(leftbottom_pwm, Speed_l);
    analogWrite(righttop_pwm, Speed_r);
    if (en_right_T > 6000 || en_left_B > 6000)
    {
      Setpoint[1] = Setpoint[2] = 0;
      break;
    }
    if (en_right_T < 0 || en_left_B < 0)
    {
      Setpoint[1] = Setpoint[2] = 6000;
      break;
    }
  }
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

  if (delta_time >= Time)
  {
    double error;
    if (encoder_value < Setpoint[motor_no])
    {
      error = Setpoint[motor_no] - encoder_value;
      digitalWrite(dir[motor_no], HIGH);
    }
    else
    {
      error = encoder_value - Setpoint[motor_no];
      digitalWrite(dir[motor_no], LOW);
    }
    total_error[motor_no] += error;
    if (total_error[motor_no] >= max_pwm)
      total_error[motor_no] = max_pwm;
    if (total_error[motor_no] <= min_pwm)
      total_error[motor_no] = min_pwm;

    double delta_error = error - last_error[motor_no];

    pwm_signal = Kp * error + (Ki * Time) * total_error[motor_no] + (Kd / Time) * delta_error;

    if (pwm_signal >= max_pwm)
      pwm_signal = max_pwm;
    if (pwm_signal <= min_pwm)
      pwm_signal = min_pwm;
    last_error[motor_no] = error;
    last_time[motor_no] = current_time;
  }
  return pwm_signal;
}
