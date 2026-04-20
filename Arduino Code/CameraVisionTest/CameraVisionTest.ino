#include <Wire.h>
#include <Servo.h>

// MPU6050
const int MPU_ADDR = 0x68;
float accelX, accelY, accelZ;
float pitchOffset = 0;
float currentPitch = 0;

// Servos
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;

const int BASE_PIN     = 9;
const int SHOULDER_PIN = 10;
const int ELBOW_PIN    = 11;
const int WRIST_PIN    = 12;

int baseAngle     = 90;
int shoulderAngle = 90;
int elbowAngle    = 90;
int wristCenter   = 90;

const int CALIBRATE_BTN = 2;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(100);

  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  wristServo.attach(WRIST_PIN);

  baseServo.write(baseAngle);
  shoulderServo.write(shoulderAngle);
  elbowServo.write(elbowAngle);
  wristServo.write(wristCenter);

  pinMode(CALIBRATE_BTN, INPUT_PULLUP);

  delay(500);
  calibrate();

  Serial.println("=== Arm Controller Ready ===");
  Serial.println("Format: <servo> <angle>");
  Serial.println("  1=Base  2=Shoulder  3=Elbow");
  Serial.println("  Wrist is auto-leveled");
  Serial.println("Type 'cal' to re-zero IMU");
  Serial.println("Type 'status' for current angles");
  Serial.println("============================\n");
}

// ---------- MPU6050 ----------

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  accelX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accelY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accelZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
}

float getPitch() {
  return atan2(accelX, accelZ) * 180.0 / PI;
}

void calibrate() {
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

// ---------- Wrist Leveling ----------

void updateWrist() {
  readMPU();
  currentPitch = getPitch() - pitchOffset;
  int wristAngle = wristCenter + (int)currentPitch;
  wristAngle = constrain(wristAngle, 0, 180);
  wristServo.write(wristAngle);
}

// ---------- Serial Commands ----------

void handleSerial() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.equalsIgnoreCase("cal")) {
    Serial.println("Re-calibrating...");
    calibrate();
    return;
  }

  if (input.equalsIgnoreCase("status")) {
    printStatus();
    return;
  }

  // BASE LEFT and BASE RIGHT from Python
  if (input.startsWith("BASE LEFT")) {
    int speed = input.substring(10).toInt();
    if (speed <= 0) speed = 5;
    baseAngle = constrain(baseAngle - speed, 0, 180);
    baseServo.write(baseAngle);
    Serial.print("Base Left -> ");
    Serial.println(baseAngle);
    return;
  }

  if (input.startsWith("BASE RIGHT")) {
    int speed = input.substring(11).toInt();
    if (speed <= 0) speed = 5;
    baseAngle = constrain(baseAngle + speed, 0, 180);
    baseServo.write(baseAngle);
    Serial.print("Base Right -> ");
    Serial.println(baseAngle);
    return;
  }

  // Manual servo control
  int spaceIndex = input.indexOf(' ');
  if (spaceIndex == -1) {
    Serial.println("Invalid format. Use: <servo> <angle>");
    return;
  }

  int servoNum = input.substring(0, spaceIndex).toInt();
  int angle    = input.substring(spaceIndex + 1).toInt();

  if (servoNum < 1 || servoNum > 3) {
    Serial.println("Servo must be 1-3. Wrist is automatic.");
    return;
  }
  if (angle < 0 || angle > 180) {
    Serial.println("Angle must be 0-180.");
    return;
  }

  switch (servoNum) {
    case 1:
      baseAngle = angle;
      baseServo.write(angle);
      Serial.print("Base -> ");
      break;
    case 2:
      shoulderAngle = angle;
      shoulderServo.write(angle);
      Serial.print("Shoulder -> ");
      break;
    case 3:
      elbowAngle = angle;
      elbowServo.write(angle);
      Serial.print("Elbow -> ");
      break;
  }
  Serial.print(angle);
  Serial.println(" deg");
}

void printStatus() {
  Serial.println("----- Current Angles -----");
  Serial.print("  1) Base     : "); Serial.println(baseAngle);
  Serial.print("  2) Shoulder : "); Serial.println(shoulderAngle);
  Serial.print("  3) Elbow    : "); Serial.println(elbowAngle);
  Serial.print("  4) Wrist    : "); Serial.print(wristCenter);
  Serial.print(" + pitch correction: "); Serial.println(currentPitch);
  Serial.println("--------------------------");
}

// ---------- Main Loop ----------

void loop() {
  if (digitalRead(CALIBRATE_BTN) == LOW) {
    Serial.println("Re-calibrating...");
    calibrate();
    delay(500);
  }

  handleSerial();
  updateWrist();

  delay(20);
}