#include <Wire.h>

const int MPU_ADDR = 0x68;

float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float temp;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Wake up the MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(100); // Give the sensor time to wake up

  // Verify it's awake by reading WHO_AM_I register
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  byte whoAmI = Wire.read();

  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI == 0x68) {
    Serial.println("MPU6050 confirmed and ready!");
  } else {
    Serial.println("Unexpected response — check sensor.");
  }

  Serial.println("==============");
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  accelX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accelY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accelZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
  temp   = (Wire.read() << 8 | Wire.read()) / 340.0 + 36.53;
  gyroX  = (Wire.read() << 8 | Wire.read()) / 131.0;
  gyroY  = (Wire.read() << 8 | Wire.read()) / 131.0;
  gyroZ  = (Wire.read() << 8 | Wire.read()) / 131.0;

  Serial.print("Accel X: "); Serial.print(accelX);
  Serial.print("  Y: ");     Serial.print(accelY);
  Serial.print("  Z: ");     Serial.println(accelZ);

  Serial.print("Gyro  X: "); Serial.print(gyroX);
  Serial.print("  Y: ");     Serial.print(gyroY);
  Serial.print("  Z: ");     Serial.println(gyroZ);

  Serial.print("Temp: ");    Serial.print(temp);
  Serial.println(" C");
  Serial.println("---");

  delay(500);
}