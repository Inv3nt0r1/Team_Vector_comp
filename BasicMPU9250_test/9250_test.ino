#include <Mpu_9250_without_serial.h>


MPU_9250 mpu;
int gyro_value;
void setup() {
Serial.begin(115200);
mpu.Start_gyro();
}

void loop() {
  mpu.compute(&gyro_value);
  Serial.print("gyro_value :\t");
  Serial.print(gyro_value);
  Serial.println("\n");

}
