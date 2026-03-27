#include "MeAuriga.h"

MeGyro gyro(0, 0x69); // For Auriga onboard gyro

void setup() {
  Serial.begin(115200);
  gyro.begin();
}

void loop() {
  gyro.update();
  
  // Read gyroscope angles (degrees)
  Serial.print("Angles - X: "); Serial.print(gyro.getAngleX(), 2);
  Serial.print(" Y: "); Serial.print(gyro.getAngleY(), 2);
  Serial.print(" Z: "); Serial.println(gyro.getAngleZ(), 2);
  
  // Read accelerations (g units)
  Serial.print("Accel - X: "); Serial.print(gyro.getAccX(), 3);
  Serial.print(" Y: "); Serial.print(gyro.getAccY(), 3);
  Serial.print(" Z: "); Serial.println(gyro.getAccZ(), 3);
  
  // Read temperature
  Serial.print("Temperature: "); Serial.print(gyro.getTemperature(), 1);
  Serial.println(" °C");
  
  delay(100);
}