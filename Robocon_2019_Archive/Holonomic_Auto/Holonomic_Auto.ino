#define leftfront_pwm 23
#define leftback_pwm 22
#define rightfront_pwm 21
#define rightback_pwm 20
#define leftfront_dir 19
#define leftback_dir 18
#define rightfront_dir 17
#define rightback_dir 16

bool clockwise[4];
int Speed = 40;
void setup() {
  pinMode(leftfront_pwm, OUTPUT);
  pinMode(leftback_pwm, OUTPUT);
  pinMode(rightfront_pwm, OUTPUT);
  pinMode(rightback_pwm, OUTPUT);
  pinMode(leftfront_dir, OUTPUT);
  pinMode(leftback_dir, OUTPUT);
  pinMode(rightfront_dir, OUTPUT);
  pinMode(rightback_dir, OUTPUT);
  Serial.begin(9600);
  for (int i = 0; i < 4; i++)
    clockwise[i] = HIGH;
  stay();
  delay(10000);
}

void loop() {
  move_front();
  delay(2000);
  stay();
  delay(1000);
  move_diagonally_right();
  delay(2000);
  stay();
  delay(1000);
  move_diagonally_left();
  delay(2000);
  stay();
  delay(1000);
}
void stay()
{
  analogWrite(rightfront_pwm, 0);
  analogWrite(rightback_pwm, 0);
  analogWrite(leftfront_pwm, 0);
  analogWrite(leftback_pwm, 0);
  Serial.println("Stop ");
}
void move_right()
{
  digitalWrite(leftfront_dir, clockwise[0]);
  digitalWrite(leftback_dir, clockwise[1]);
  digitalWrite(rightfront_dir, clockwise[2]);
  digitalWrite(rightback_dir, !clockwise[3]);
  analogWrite(leftfront_pwm, Speed);
  analogWrite(leftback_pwm, Speed+15);
  analogWrite(rightfront_pwm, Speed);
  analogWrite(rightback_pwm, Speed+20);
  Serial.println("bot right");
}
void move_left()
{
  digitalWrite(leftfront_dir, !clockwise[0]);
  digitalWrite(leftback_dir, !clockwise[1]);
  digitalWrite(rightfront_dir, !clockwise[2]);
  digitalWrite(rightback_dir, clockwise[3]);
  analogWrite(leftfront_pwm, Speed);
  analogWrite(leftback_pwm, Speed+15);
  analogWrite(rightfront_pwm, Speed);
  analogWrite(rightback_pwm, Speed+20);
  Serial.println("bot left");
}
void move_front()
{
  digitalWrite(leftfront_dir, clockwise[0]);
  digitalWrite(leftback_dir, !clockwise[1]);
  digitalWrite(rightfront_dir, !clockwise[2]);
  digitalWrite(rightback_dir, !clockwise[3]);
  analogWrite(leftfront_pwm, Speed);
  analogWrite(leftback_pwm, Speed+5);
  analogWrite(rightback_pwm, Speed);
  analogWrite(rightfront_pwm, Speed+5);
  Serial.println("bot front");
}
void move_back()
{
  digitalWrite(leftfront_dir, !clockwise[0]);
  digitalWrite(leftback_dir, clockwise[1]);
  digitalWrite(rightfront_dir, clockwise[2]);
  digitalWrite(rightback_dir, clockwise[3]);
  analogWrite(leftfront_pwm, Speed);
  analogWrite(leftback_pwm, Speed+15);
  analogWrite(rightback_pwm, Speed);
  analogWrite(rightfront_pwm, Speed+20);
  Serial.println("bot back");
}
void move_diagonally_right()
{
  digitalWrite(leftfront_dir, clockwise[0]);
  digitalWrite(rightback_dir, !clockwise[3]);
  analogWrite(leftfront_pwm, Speed);
  analogWrite(rightback_pwm, Speed+5);
  Serial.println("bot diagonaly right");
}
void move_diagonally_left()
{
  digitalWrite(leftback_dir, !clockwise[1]);
  digitalWrite(rightfront_dir, !clockwise[2]);
  analogWrite(leftback_pwm, Speed+5);
  analogWrite(rightfront_pwm, Speed);
  Serial.println("bot diagonaly left");
}
void move_clockwise()
{
  digitalWrite(leftfront_dir, clockwise[0]);
  digitalWrite(leftback_dir, !clockwise[1]);
  digitalWrite(rightfront_dir, clockwise[2]);
  digitalWrite(rightback_dir, clockwise[3]);
  analogWrite(leftfront_pwm, Speed);
  analogWrite(leftback_pwm, Speed);
  analogWrite(rightback_pwm, Speed);
  analogWrite(rightfront_pwm, Speed);
  Serial.println("bot clockwise");
}
void move_anticlockwise()
{
  digitalWrite(leftfront_dir, !clockwise[0]);
  digitalWrite(leftback_dir, clockwise[1]);
  digitalWrite(rightfront_dir, !clockwise[2]);
  digitalWrite(rightback_dir, !clockwise[3]);
  analogWrite(leftfront_pwm, Speed);
  analogWrite(leftback_pwm, Speed);
  analogWrite(rightback_pwm, Speed);
  analogWrite(rightfront_pwm, Speed);
  Serial.println("bot anticlockwise");
}
