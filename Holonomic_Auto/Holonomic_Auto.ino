#define left_top_pwm 23
#define left_bottom_pwm 22
#define right_top_pwm 21
#define right_bottom_pwm 20
#define left_top_dir 19
#define left_bottom_dir 18
#define right_top_dir 17
#define right_bottom_dir 16

bool clockwise = HIGH;
int Speed = 40;
void setup() {
  pinMode(left_top_pwm, OUTPUT);
  pinMode(left_top_dir, OUTPUT);
  pinMode(left_bottom_pwm, OUTPUT);
  pinMode(left_bottom_dir, OUTPUT);
  pinMode(right_top_pwm, OUTPUT);
  pinMode(right_top_dir, OUTPUT);
  pinMode(right_bottom_pwm, OUTPUT);
  pinMode(right_bottom_dir, OUTPUT);
  Serial.begin(9600);
  delay(10000);
}

void loop() {
  unsigned long Time = 0;
  unsigned long last_time = 0;
  while (Time - last_time < 2000)
  {
    diagonal_left(true);
    Time = millis();
  }
  last_time = Time;
  while (Time - last_time < 2000)
  {
    diagonal_right(true);
    Time = millis();
  }
  last_time = Time;
  while (Time - last_time < 2000)
  {
    diagonal_left(true);
    Time = millis();
  }
  last_time = Time;
  while (Time - last_time < 2000)
  {
    diagonal_right(true);
    Time = millis();
  }
}
void forward()
{
  digitalWrite(left_top_dir, clockwise);
  digitalWrite(right_top_dir, !clockwise);
  digitalWrite(left_bottom_dir, clockwise);
  digitalWrite(right_bottom_dir, !clockwise);

  Serial.println("Forward");
  digitalWrite(left_top_pwm, Speed);
  digitalWrite(left_bottom_pwm, Speed);
  digitalWrite(right_top_pwm, Speed);
  digitalWrite(right_bottom_pwm, Speed);
}
void backward()
{
  digitalWrite(left_top_dir, !clockwise);
  digitalWrite(right_top_dir, clockwise);
  digitalWrite(left_bottom_dir, !clockwise);
  digitalWrite(right_bottom_dir, clockwise);

  Serial.println("Backward");
  digitalWrite(left_top_pwm, Speed);
  digitalWrite(left_bottom_pwm, Speed);
  digitalWrite(right_top_pwm, Speed);
  digitalWrite(right_bottom_pwm, Speed);
}
void left()
{
  digitalWrite(left_top_dir, !clockwise);
  digitalWrite(right_top_dir, !clockwise);
  digitalWrite(left_bottom_dir, clockwise);
  digitalWrite(right_bottom_dir, clockwise);

  Serial.println("Left");
  digitalWrite(left_top_pwm, Speed);
  digitalWrite(left_bottom_pwm, Speed);
  digitalWrite(right_top_pwm, Speed);
  digitalWrite(right_bottom_pwm, Speed);
}
void right()
{
  digitalWrite(left_top_dir, clockwise);
  digitalWrite(right_top_dir, clockwise);
  digitalWrite(left_bottom_dir, !clockwise);
  digitalWrite(right_bottom_dir, !clockwise);

  Serial.println("Right");
  digitalWrite(left_top_pwm, Speed);
  digitalWrite(left_bottom_pwm, Speed);
  digitalWrite(right_top_pwm, Speed);
  digitalWrite(right_bottom_pwm, Speed);
}
void diagonal_left(bool flag)
{
  if (flag)
  {
    digitalWrite(left_top_dir, clockwise);
    digitalWrite(right_top_dir, clockwise);
    digitalWrite(left_bottom_dir, !clockwise);
    digitalWrite(right_bottom_dir, !clockwise);

    Serial.println("Diagonal Left Forward");
    digitalWrite(left_top_pwm, 0);
    digitalWrite(left_bottom_pwm, Speed);
    digitalWrite(right_top_pwm, Speed);
    digitalWrite(right_bottom_pwm, 0);
  }
  if (!flag)
  {
    digitalWrite(left_top_dir, !clockwise);
    digitalWrite(right_top_dir, !clockwise);
    digitalWrite(left_bottom_dir, clockwise);
    digitalWrite(right_bottom_dir, clockwise);

    Serial.println("Diagonal Left Backward");
    digitalWrite(left_top_pwm, 0);
    digitalWrite(left_bottom_pwm, Speed);
    digitalWrite(right_top_pwm, Speed);
    digitalWrite(right_bottom_pwm, 0);
  }
}
void diagonal_right(bool flag)
{
  if (flag)
  {
    digitalWrite(left_top_dir, clockwise);
    digitalWrite(right_top_dir, clockwise);
    digitalWrite(left_bottom_dir, !clockwise);
    digitalWrite(right_bottom_dir, !clockwise);

    Serial.println("Diagonal Right Forward");
    digitalWrite(left_top_pwm, Speed);
    digitalWrite(left_bottom_pwm, 0);
    digitalWrite(right_top_pwm, 0);
    digitalWrite(right_bottom_pwm, Speed);
  }
  if (!flag)
  {
    digitalWrite(left_top_dir, !clockwise);
    digitalWrite(right_top_dir, !clockwise);
    digitalWrite(left_bottom_dir, clockwise);
    digitalWrite(right_bottom_dir, clockwise);

    Serial.println("Diagonal Right Backward");
    digitalWrite(left_top_pwm, Speed);
    digitalWrite(left_bottom_pwm, 0);
    digitalWrite(right_top_pwm, 0);
    digitalWrite(right_bottom_pwm, Speed);
  }
}
