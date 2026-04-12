#include <Wire.h>
#include <Servo.h>

const int MPU_ADDR = 0x68;
const int WRIST_PIN = 9;
const int CALIBRATE_BTN = 2; // Optional: button to re-zero

Servo wristServo;

float accelX, accelY, accelZ;
float pitchOffset = 0;
float currentPitch = 0;
int wristCenter = 90; // Servo center position

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(100);

  wristServo.attach(WRIST_PIN);
  wristServo.write(wristCenter);

  pinMode(CALIBRATE_BTN, INPUT_PULLUP);

  delay(500); // Let sensor settle
  calibrate();

  Serial.println("Wrist Leveling Active");
  Serial.println("Type 'cal' to re-zero");
  Serial.println("======================");
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); // Only need accel data

  accelX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accelY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accelZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
}

float getPitch() {
  return atan2(accelX, accelZ) * 180.0 / PI;
}

void calibrate() {
  // Average multiple readings for a stable offset
  float sum = 0;
  for (int i = 0; i < 50; i++) {
    readMPU();
    sum += getPitch();
    delay(10);
  }
  pitchOffset = sum / 50.0;
  Serial.print("Calibrated. Offset: ");
  Serial.println(pitchOffset);
}

void loop() {
  // Check for calibration button
  if (digitalRead(CALIBRATE_BTN) == LOW) {
    Serial.println("Re-calibrating...");
    calibrate();
    delay(500); // Debounce
  }

  // Check for serial calibration command
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equalsIgnoreCase("cal")) {
      Serial.println("Re-calibrating...");
      calibrate();
    }
  }

  // Read current tilt
  readMPU();
  currentPitch = getPitch() - pitchOffset;

  // Directly map tilt to servo correction
  int servoAngle = wristCenter + (int)currentPitch;
  servoAngle = constrain(servoAngle, 0, 180);

  wristServo.write(servoAngle);

  Serial.print("Pitch: ");
  Serial.print(currentPitch);
  Serial.print("  Servo: ");
  Serial.println(servoAngle);

  delay(20); // 50Hz update rate
}