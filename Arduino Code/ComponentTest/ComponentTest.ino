// PickMasters — ComponentTest
// Interactive bench test for every component, using the CURRENT hardware setup
// (standalone L298N motor driver + servos on digital pins).
//
// Open the Serial Monitor at 9600 baud, set line ending to "Newline" (or just
// send single characters), and press a key to run each test:
//
//   1  — Shoulder servo sweep  (pin 9)
//   2  — Elbow servo sweep     (pin 10)
//   3  — Wrist servo sweep     (pin 11)
//   4  — Hand servo sweep      (pin 12)
//   5  — All servos sweep together
//   p  — Pump relay ON/OFF toggle      (pin 3, ACTIVE LOW)
//   l  — Left motor: FWD -> REV -> STOP (ENA=5, IN1=2, IN2=4 -> OUT1/OUT2)
//   r  — Right motor: FWD -> REV -> STOP (ENB=6, IN3=7, IN4=8 -> OUT3/OUT4)
//   m  — Both motors: FWD -> REV -> PIVOT L -> PIVOT R -> STOP
//   i  — Stream IMU pitch for 5 s         (MPU6050 on A4/A5)
//   b  — Watch calibration button for 5 s (pin 13, INPUT_PULLUP)
//   h  — Print this menu again

#include <Wire.h>
#include <Servo.h>

// ── Servo pins ───────────────────────────────────────────────────────────────
#define SHOULDER_PIN  9
#define ELBOW_PIN     10
#define WRIST_PIN     11
#define HAND_PIN      12

// ── Pump relay (active LOW) ──────────────────────────────────────────────────
#define PUMP_PIN      3

// ── IMU calibration button ───────────────────────────────────────────────────
#define CAL_BUTTON    13

// ── L298N motor driver ───────────────────────────────────────────────────────
#define ENA  5     // PWM speed, left wheel
#define IN1  2     // direction A, left wheel
#define IN2  4     // direction B, left wheel
#define ENB  6     // PWM speed, right wheel
#define IN3  7     // direction A, right wheel
#define IN4  8     // direction B, right wheel
const int DRIVE_SPEED = 200;

// ── IMU ──────────────────────────────────────────────────────────────────────
#define MPU_ADDR      0x68

Servo shoulderServo, elbowServo, wristServo, handServo;
bool pumpOn = false;

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // Pump relay — active LOW, HIGH = OFF at startup
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, HIGH);

  pinMode(CAL_BUTTON, INPUT_PULLUP);

  // L298N control pins
  pinMode(ENA, OUTPUT);  pinMode(IN1, OUTPUT);  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);  pinMode(IN3, OUTPUT);  pinMode(IN4, OUTPUT);
  stopMotors();

  // Servos centre
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  wristServo.attach(WRIST_PIN);
  handServo.attach(HAND_PIN);
  shoulderServo.write(90);
  elbowServo.write(90);
  wristServo.write(90);
  handServo.write(90);

  // Wake up MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(200);

  Serial.println(F("PickMasters component test ready"));
  printMenu();
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  if (!Serial.available()) return;
  char c = (char)Serial.read();

  switch (c) {
    case '1': sweepServo(shoulderServo, "Shoulder"); break;
    case '2': sweepServo(elbowServo,    "Elbow");    break;
    case '3': sweepServo(wristServo,    "Wrist");    break;
    case '4': sweepServo(handServo,     "Hand");     break;
    case '5': sweepAllServos();                      break;
    case 'p': togglePump();                          break;
    case 'l': testMotorLeft();                       break;
    case 'r': testMotorRight();                      break;
    case 'm': testBothMotors();                      break;
    case 'i': streamIMU();                           break;
    case 'b': watchButton();                         break;
    case 'h': printMenu();                           break;
    case '\n': case '\r': break;   // ignore line endings
    default:
      Serial.print(F("Unknown command: "));
      Serial.println(c);
      break;
  }
}

// ── Servo tests ───────────────────────────────────────────────────────────────
void sweepServo(Servo &s, const char *name) {
  Serial.print(F("Sweeping "));
  Serial.print(name);
  Serial.println(F(" 0 -> 180 -> 90"));
  for (int a = 0; a <= 180; a += 2) { s.write(a); delay(15); }
  for (int a = 180; a >= 0; a -= 2) { s.write(a); delay(15); }
  s.write(90);
  Serial.println(F("  done (returned to 90)"));
}

void sweepAllServos() {
  Serial.println(F("Sweeping ALL servos 0 -> 180 -> 90"));
  for (int a = 0; a <= 180; a += 2) {
    shoulderServo.write(a); elbowServo.write(a);
    wristServo.write(a);    handServo.write(a);
    delay(15);
  }
  for (int a = 180; a >= 0; a -= 2) {
    shoulderServo.write(a); elbowServo.write(a);
    wristServo.write(a);    handServo.write(a);
    delay(15);
  }
  shoulderServo.write(90); elbowServo.write(90);
  wristServo.write(90);    handServo.write(90);
  Serial.println(F("  done (all returned to 90)"));
}

// ── Pump test ─────────────────────────────────────────────────────────────────
void togglePump() {
  pumpOn = !pumpOn;
  digitalWrite(PUMP_PIN, pumpOn ? LOW : HIGH);   // active LOW
  Serial.print(F("Pump "));
  Serial.println(pumpOn ? F("ON") : F("OFF"));
}

// ── Motor tests ───────────────────────────────────────────────────────────────
void setMotorLeft(int dir) {
  if (dir > 0)      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, DRIVE_SPEED); }
  else if (dir < 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, DRIVE_SPEED); }
  else              { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  analogWrite(ENA, 0); }
}

void setMotorRight(int dir) {
  if (dir > 0)      { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, DRIVE_SPEED); }
  else if (dir < 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, DRIVE_SPEED); }
  else              { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  analogWrite(ENB, 0); }
}

void stopMotors() { setMotorLeft(0); setMotorRight(0); }

void testMotorLeft() {
  Serial.println(F("LEFT motor (OUT1/OUT2): FWD 1.5s"));
  setMotorLeft(1);  delay(1500);
  Serial.println(F("  REV 1.5s"));
  setMotorLeft(-1); delay(1500);
  setMotorLeft(0);
  Serial.println(F("  STOP"));
}

void testMotorRight() {
  Serial.println(F("RIGHT motor (OUT3/OUT4): FWD 1.5s"));
  setMotorRight(1);  delay(1500);
  Serial.println(F("  REV 1.5s"));
  setMotorRight(-1); delay(1500);
  setMotorRight(0);
  Serial.println(F("  STOP"));
}

void testBothMotors() {
  Serial.println(F("BOTH: forward 1.5s"));
  setMotorLeft(1);  setMotorRight(1);  delay(1500);
  Serial.println(F("  reverse 1.5s"));
  setMotorLeft(-1); setMotorRight(-1); delay(1500);
  Serial.println(F("  pivot LEFT 1.5s"));
  setMotorLeft(-1); setMotorRight(1);  delay(1500);
  Serial.println(F("  pivot RIGHT 1.5s"));
  setMotorLeft(1);  setMotorRight(-1); delay(1500);
  stopMotors();
  Serial.println(F("  STOP"));
}

// ── IMU test ──────────────────────────────────────────────────────────────────
float readPitch() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  float accelX = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  float accelY = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  float accelZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  (void)accelY;
  return atan2(accelX, accelZ) * 180.0 / PI;
}

void streamIMU() {
  Serial.println(F("Streaming IMU pitch for 5 s — tilt the arm..."));
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    Serial.print(F("  pitch = "));
    Serial.print(readPitch(), 1);
    Serial.println(F(" deg"));
    delay(200);
  }
  Serial.println(F("  done"));
}

// ── Calibration button test ──────────────────────────────────────────────────
void watchButton() {
  Serial.println(F("Watching cal button (pin 13) for 5 s — press it..."));
  bool prev = HIGH;
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    bool btn = digitalRead(CAL_BUTTON);
    if (btn == LOW && prev == HIGH) Serial.println(F("  BUTTON PRESSED"));
    prev = btn;
    delay(10);
  }
  Serial.println(F("  done"));
}

// ── Menu ──────────────────────────────────────────────────────────────────────
void printMenu() {
  Serial.println(F("------ component test menu ------"));
  Serial.println(F(" 1  Shoulder servo (pin 9)"));
  Serial.println(F(" 2  Elbow servo    (pin 10)"));
  Serial.println(F(" 3  Wrist servo    (pin 11)"));
  Serial.println(F(" 4  Hand servo     (pin 12)"));
  Serial.println(F(" 5  All servos together"));
  Serial.println(F(" p  Pump relay toggle (pin 3)"));
  Serial.println(F(" l  Left motor  (OUT1/OUT2)"));
  Serial.println(F(" r  Right motor (OUT3/OUT4)"));
  Serial.println(F(" m  Both motors sequence"));
  Serial.println(F(" i  IMU pitch stream (5 s)"));
  Serial.println(F(" b  Cal button watch (5 s)"));
  Serial.println(F(" h  Show this menu"));
  Serial.println(F("--------------------------------"));
}
