const byte ledPin = 13;
const byte interruptPin = 2;
volatile byte state = LOW;

void setup() {
pinMode(ledPin, OUTPUT);
attachInterrupt(digitalPinToInterrupt(interruptPin), test, CHANGE);
Serial.begin(9600);
}

void loop() {
digitalWrite(ledPin, state);
}

void test() {
state=!state;
Serial.println("Detect!");
}
