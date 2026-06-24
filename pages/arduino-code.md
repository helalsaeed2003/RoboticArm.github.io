# Development Sketches

[← Back to Home](../index.md)

---

These are the development-stage Arduino and Processing sketches created during the build process. They test individual components, iterate on control approaches, and document the evolution from basic servo tests to the final integrated firmware. The production code is in [Final Code](final-code.md).

---

## `ArmController.ino`

An earlier version of the manual-mode firmware before PID wrist levelling and the safety interlock were added. Drives servos and DC motors via serial commands from the Processing gamepad sketch. Used during mid-project integration testing.

<details>
<summary>Arduino C | 267 lines</summary>
<div class="code-scroll">

```cpp
// PickMasters — ArmController firmware
// 4-DOF robotic arm, manual control over USB serial (paired with DriveControl.pde)
//
// Hardware: Arduino Uno + standalone L298N dual H-bridge motor driver
//
// Pin usage:
//   9   — Shoulder servo
//   10  — Elbow servo
//   11  — Wrist servo
//   12  — Hand servo
//   A4  — IMU SDA (I2C, reserved by Wire)
//   A5  — IMU SCL (I2C, reserved by Wire)
//   3   — Pump relay (ACTIVE LOW: LOW = on, HIGH = off, starts off)
//   L298N driver: ENA=5, IN1=2, IN2=4 (left wheel);
//                 ENB=6, IN3=7, IN4=8 (right wheel).
//
// Serial protocol (9600 baud, newline-terminated, ONE combined message per frame):
//   S<shoulder>,<elbow>,<wrist>,<hand>,M<left>,<right>,P<0|1>,W<0|1>
//     servo angles 0..180, motor directions -1/0/1, P1 = pump on,
//     W1 = wrist AUTO (IMU leveling), W0 = wrist MANUAL
//   cal     — re-zero IMU pitch offset (sent by the DriveControl CalButton)
//   status  — print all current angles and states (single compact line)

#include <Wire.h>
#include <Servo.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
#define SHOULDER_PIN  9
#define ELBOW_PIN     10
#define WRIST_PIN     11
#define HAND_PIN      12
#define PUMP_PIN      3     // relay is active LOW
#define MPU_ADDR      0x68

// ── DC motors via L298N dual H-bridge ────────────────────────────────────────
#define ENA  5     // PWM speed, left wheel
#define IN1  2     // direction A, left wheel
#define IN2  4     // direction B, left wheel
#define ENB  6     // PWM speed, right wheel
#define IN3  7     // direction A, right wheel
#define IN4  8     // direction B, right wheel
const int DRIVE_SPEED = 200;   // single constant speed when a motor is active

// ── Servos ───────────────────────────────────────────────────────────────────
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo handServo;

int shoulderAngle = 90;
int elbowAngle    = 90;
int wristAngle    = 90;
int handAngle     = 90;

int  leftDir  = 0;            // -1 / 0 / +1
int  rightDir = 0;
bool pumpOn   = false;
bool wristAutoMode = true;    // true = IMU leveling, false = Processing controls wrist

// ── IMU state ────────────────────────────────────────────────────────────────
bool  imuOk        = false;   // false = MPU6050 not detected; auto-level disabled
float pitchOffset  = 0.0;
float currentPitch = 0.0;

unsigned long lastIMU    = 0;

// ── Serial receive buffer ─────────────────────────────────────────────────────
char serialBuf[64];
byte serialLen = 0;

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Serial.write("PickMasters booting\n");   // instant proof serial is alive

  // Pump relay — active LOW, so HIGH = OFF at startup
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, HIGH);

  // L298N control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Servos — centre on startup, one at a time to spread out the inrush current
  shoulderServo.attach(SHOULDER_PIN); shoulderServo.write(shoulderAngle); delay(100);
  elbowServo.attach(ELBOW_PIN);       elbowServo.write(elbowAngle);       delay(100);
  wristServo.attach(WRIST_PIN);       wristServo.write(wristAngle);       delay(100);
  handServo.attach(HAND_PIN);         handServo.write(handAngle);         delay(100);

  // Motors idle
  setMotorLeft(0);
  setMotorRight(0);

  Wire.begin();
  Wire.setWireTimeout(3000, true);

  Wire.beginTransmission(MPU_ADDR);
  imuOk = (Wire.endTransmission(true) == 0);

  if (imuOk) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);   // PWR_MGMT_1
    Wire.write(0x00);   // clear sleep bit
    Wire.endTransmission(true);
    delay(200);
    calibrateIMU();
  } else {
    wristAutoMode = false;
    Serial.write("WARN: IMU not found — wrist auto-level disabled\n");
  }

  Serial.write("PickMasters ready\n");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  handleSerial();

  if (imuOk && millis() - lastIMU >= 20) {   // 50 Hz IMU / wrist update
    lastIMU = millis();
    currentPitch = readPitch();
    if (wristAutoMode) {
      wristAngle = constrain((int)(90 + (currentPitch - pitchOffset)), 0, 180);
      wristServo.write(wristAngle);
    }
  }
}

// ── Serial parsing ────────────────────────────────────────────────────────────
void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serialBuf[serialLen] = '\0';
      parseCommand(serialBuf);
      serialLen = 0;
    } else if (c != '\r' && serialLen < sizeof(serialBuf) - 1) {
      serialBuf[serialLen++] = c;
    }
  }
}

void parseCommand(char *cmd) {
  if (cmd[0] == '\0') return;

  if (cmd[0] == 'S') {
    int sh, el, wr, ha, l, r, p, w;
    if (sscanf(cmd, "S%d,%d,%d,%d,M%d,%d,P%d,W%d",
               &sh, &el, &wr, &ha, &l, &r, &p, &w) == 8) {
      shoulderAngle = constrain(sh, 0, 180);
      elbowAngle    = constrain(el, 0, 180);
      handAngle     = constrain(ha, 0, 180);
      shoulderServo.write(shoulderAngle);
      elbowServo.write(elbowAngle);
      handServo.write(handAngle);

      wristAutoMode = (w != 0);
      if (!wristAutoMode) {
        wristAngle = constrain(wr, 0, 180);
        wristServo.write(wristAngle);
      }

      setMotorLeft(l);
      setMotorRight(r);
      leftDir  = l;
      rightDir = r;

      pumpOn = (p != 0);
      digitalWrite(PUMP_PIN, pumpOn ? LOW : HIGH);
    }

  } else if (strcmp(cmd, "cal") == 0) {
    calibrateIMU();
    Serial.write("cal ok\n");

  } else if (strcmp(cmd, "status") == 0) {
    printStatus();
  }
}

// ── DC motor control (L298N) ──────────────────────────────────────────────────
void setMotorLeft(int dir) {
  if (dir > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, DRIVE_SPEED);
  } else if (dir < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, DRIVE_SPEED);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void setMotorRight(int dir) {
  if (dir > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, DRIVE_SPEED);
  } else if (dir < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, DRIVE_SPEED);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

// ── IMU wrist auto-level ──────────────────────────────────────────────────────
float readPitch() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  if (Wire.requestFrom(MPU_ADDR, 6, true) != 6) return currentPitch;

  float accelX = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  float accelY = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  float accelZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  (void)accelY;

  return atan2(accelX, accelZ) * 180.0 / PI;
}

void calibrateIMU() {
  float sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += readPitch();
    delay(10);
  }
  pitchOffset = sum / 50.0;
}

// ── Status report (single compact line) ──────────────────────────────────────
void printStatus() {
  Serial.print("S:");    Serial.print(shoulderAngle);
  Serial.print(" E:");   Serial.print(elbowAngle);
  Serial.print(" W:");   Serial.print(wristAngle);
  Serial.print(" H:");   Serial.print(handAngle);
  Serial.print(" M:");   Serial.print(leftDir);
  Serial.print(",");     Serial.print(rightDir);
  Serial.print(" P:");   Serial.print(pumpOn ? 1 : 0);
  Serial.print(" MODE:"); Serial.print(wristAutoMode ? "AUTO" : "MAN");
  Serial.print(" IMU:");  Serial.print(imuOk ? "OK" : "NONE");
  Serial.print(" PITCH:"); Serial.print(currentPitch - pitchOffset, 1);
  Serial.write('\n');
}
```

</div>
</details>

---

## `ComponentTest.ino`

Interactive bench-test sketch for every hardware component. Open the Serial Monitor, type a command, and individually test each servo, DC motor direction, pump relay, and IMU reading. Essential for verifying wiring before running the full firmware.

<details>
<summary>Arduino C | 251 lines</summary>
<div class="code-scroll">

```cpp
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
//   b  — Watch calibration button for 5 s (pin A0, INPUT_PULLUP)
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
#define CAL_BUTTON    A0

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
  Serial.println(F("Watching cal button (pin A0) for 5 s — press it..."));
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
  Serial.println(F(" b  Cal button watch (pin A0, 5 s)"));
  Serial.println(F(" h  Show this menu"));
  Serial.println(F("--------------------------------"));
}
```

</div>
</details>

---

## `AutoMode.ino`

Early automatic-mode prototype. Combines servo control with basic serial commands from the vision script. Predates the fuzzy-logic controller — uses fixed-speed motor commands instead.

<details>
<summary>Arduino C | 184 lines</summary>
<div class="code-scroll">

```cpp
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

// Calibration button (optional)
const int CALIBRATE_BTN = 2;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Wake up MPU6050
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
```

</div>
</details>

---

## `CameraVisionTest.ino`

Test firmware for validating the camera-to-Arduino communication loop. Accepts vision commands over serial and drives servos and motors in response. Used to debug the serial protocol between `detect_and_move.py` and the Arduino.

<details>
<summary>Arduino C | 203 lines</summary>
<div class="code-scroll">

```cpp
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
```

</div>
</details>

---

## `ManualDrive.ino`

The original manual-drive firmware written for the L293D motor shield (before it shorted). Uses the AFMotor library. Kept in the repo as a record of the original hardware design and the lesson learned about driver current ratings.

<details>
<summary>Arduino C | 224 lines</summary>
<div class="code-scroll">

```cpp
// PickMasters — ManualDrive firmware
// Hardware: Arduino + L293D Motor Shield (Adafruit v1 / AFMotor library)
//
// Pin usage (analog pins consumed first per design requirement):
//   A0  — Pump relay (digital output)
//   A1  — Shoulder servo
//   A2  — Elbow servo
//   A3  — Wrist servo
//   A4  — IMU SDA  (I2C, reserved by Wire)
//   A5  — IMU SCL  (I2C, reserved by Wire)
//   Pin 2 — IMU calibration button (INPUT_PULLUP)
//   Motor shield occupies pins 3,4,5,6,7,8,11,12 internally.
//   Remaining free PWM: 9, 10, 13.
//
// Serial protocol (9600 baud, newline-terminated):
//   M <left> <right>   — DC motor speeds, -255..255  (differential drive)
//   2 <angle>          — Shoulder servo, 0..180 deg
//   3 <angle>          — Elbow servo,    0..180 deg
//   P 1 / P 0          — Pump relay ON / OFF
//   cal                — Re-zero IMU pitch
//   stop               — Stop motors
//   status             — Print all joint states

#include <Wire.h>
#include <Servo.h>
#include <AFMotor.h>   // Adafruit Motor Shield v1 library

// ── Pin definitions ──────────────────────────────────────────────────────────
#define PUMP_PIN      A0
#define SHOULDER_PIN  A1
#define ELBOW_PIN     A2
#define WRIST_PIN     A3
#define CAL_BUTTON    2     // INPUT_PULLUP — press to re-zero IMU

// ── DC motors via L293D motor shield ─────────────────────────────────────────
AF_DCMotor motorLeft(1);
AF_DCMotor motorRight(2);

// ── Servos ───────────────────────────────────────────────────────────────────
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;

int shoulderAngle = 90;
int elbowAngle    = 90;
int wristAngle    = 90;

// ── IMU (MPU6050, I2C address 0x68) ──────────────────────────────────────────
#define MPU_ADDR 0x68

float pitchOffset  = 0.0;
float wristCenter  = 90.0;

unsigned long lastIMU    = 0;
bool          prevCalBtn = HIGH;  // HIGH = not pressed (PULLUP)

// ── Serial receive buffer ─────────────────────────────────────────────────────
String serialBuf = "";

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // Pump relay — default OFF
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  // Calibration button
  pinMode(CAL_BUTTON, INPUT_PULLUP);

  // Servos — centre on startup
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  wristServo.attach(WRIST_PIN);
  shoulderServo.write(shoulderAngle);
  elbowServo.write(elbowAngle);
  wristServo.write(wristAngle);

  // Motors idle
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);

  // Wake up MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // clear sleep bit
  Wire.endTransmission(true);
  delay(200);

  calibrateIMU();
  Serial.println("PickMasters ready");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  handleSerial();
  handleCalButton();

  if (millis() - lastIMU >= 20) {   // 50 Hz wrist update
    lastIMU = millis();
    updateWrist();
  }
}

// ── Serial parsing ────────────────────────────────────────────────────────────
void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      parseCommand(serialBuf);
      serialBuf = "";
    } else if (c != '\r') {
      serialBuf += c;
    }
  }
}

void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.startsWith("M ")) {
    int sep = cmd.indexOf(' ', 2);
    if (sep < 0) return;
    int l = cmd.substring(2, sep).toInt();
    int r = cmd.substring(sep + 1).toInt();
    driveMotors(l, r);

  } else if (cmd.startsWith("P ")) {
    bool on = (cmd.substring(2).toInt() != 0);
    digitalWrite(PUMP_PIN, on ? HIGH : LOW);
    Serial.println(on ? "Pump ON" : "Pump OFF");

  } else if (cmd.startsWith("2 ")) {
    shoulderAngle = constrain(cmd.substring(2).toInt(), 0, 180);
    shoulderServo.write(shoulderAngle);
    Serial.println("Shoulder -> " + String(shoulderAngle));

  } else if (cmd.startsWith("3 ")) {
    elbowAngle = constrain(cmd.substring(2).toInt(), 0, 180);
    elbowServo.write(elbowAngle);
    Serial.println("Elbow -> " + String(elbowAngle));

  } else if (cmd == "cal") {
    calibrateIMU();
    Serial.println("IMU calibrated");

  } else if (cmd == "stop") {
    driveMotors(0, 0);
    Serial.println("Motors stopped");

  } else if (cmd == "status") {
    Serial.println("----- Status -----");
    Serial.println("Shoulder : " + String(shoulderAngle));
    Serial.println("Elbow    : " + String(elbowAngle));
    Serial.println("Wrist    : " + String(wristAngle) + " (auto)");
    Serial.println("------------------");
  }
}

// ── DC motor control ──────────────────────────────────────────────────────────
void driveMotors(int l, int r) {
  setMotor(motorLeft,  l);
  setMotor(motorRight, r);
}

void setMotor(AF_DCMotor &m, int spd) {
  if (spd > 0) {
    m.setSpeed(min(spd, 255));
    m.run(FORWARD);
  } else if (spd < 0) {
    m.setSpeed(min(-spd, 255));
    m.run(BACKWARD);
  } else {
    m.setSpeed(0);
    m.run(RELEASE);
  }
}

// ── IMU wrist auto-level ──────────────────────────────────────────────────────
void calibrateIMU() {
  pitchOffset = readPitch();
}

float readPitch() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);

  int16_t ax = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t ay = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t az = ((int16_t)Wire.read() << 8) | Wire.read();

  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  return atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;
}

void updateWrist() {
  float pitch = readPitch() - pitchOffset;
  wristAngle = constrain((int)(wristCenter + pitch), 0, 180);
  wristServo.write(wristAngle);
}

// ── Hardware calibration button ───────────────────────────────────────────────
void handleCalButton() {
  bool btn = digitalRead(CAL_BUTTON);
  if (btn == LOW && prevCalBtn == HIGH) {  // falling edge = pressed
    calibrateIMU();
    Serial.println("IMU calibrated (button)");
  }
  prevCalBtn = btn;
}
```

</div>
</details>

---

## `SelfLevling.ino`

Standalone wrist auto-levelling test. Reads the MPU6050 over I²C, computes pitch from accelerometer data, and drives a single servo to keep the wrist level. Used to tune the PID gains before integrating into the full ArmController.

<details>
<summary>Arduino C | 100 lines</summary>
<div class="code-scroll">

```cpp
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
```

</div>
</details>

---

## `PickAndMove.ino`

Earlier version of the automatic-mode firmware. Simpler command set without the fuzzy-logic speed controller — uses fixed PWM values for each direction command.

<details>
<summary>Arduino C | 131 lines</summary>
<div class="code-scroll">

```cpp
// PickMasters — PickAndMove
// =========================
// Receives motion commands from detect_and_move.py over serial and drives the
// robot's TWO DC motors (via an L298N) so the camera centres an item inside the
// target box.
//
//   * Spinning the base   -> PIVOT the wheels (left motor and right motor turn
//                            in opposite directions) to line the item up on the
//                            VERTICAL line (X axis).
//   * Moving the base      -> DRIVE both wheels the same direction to line the
//                            item up on the HORIZONTAL line (Y axis).
//
// Only the DC motors move here — the servos are left alone.
//
// Serial protocol (one command per line, 9600 baud):
//   PIVOT_LEFT  <speed>    spin base left   (speed = PWM 0-255)
//   PIVOT_RIGHT <speed>    spin base right
//   DRIVE_FWD   <speed>    move base forward
//   DRIVE_BACK  <speed>    move base backward
//   STOP                   stop both motors
//
// Motion is PULSE based: each command runs the motors for PULSE_MS then stops
// automatically.  That way a dropped serial link can never leave the robot
// running away — the Python loop keeps sending pulses while it needs to move.

// ── L298N motor driver pins (matches the current hardware / ComponentTest) ──
#define ENA  5     // PWM speed, left wheel
#define IN1  2     // direction A, left wheel
#define IN2  4     // direction B, left wheel
#define ENB  6     // PWM speed, right wheel
#define IN3  7     // direction A, right wheel
#define IN4  8     // direction B, right wheel

const unsigned long PULSE_MS = 70;    // how long one motion pulse lasts (shorter = gentler nudges)

unsigned long stopAt = 0;             // millis() time to auto-stop (0 = stopped)

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);  pinMode(IN1, OUTPUT);  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);  pinMode(IN3, OUTPUT);  pinMode(IN4, OUTPUT);
  stopMotors();

  Serial.println(F("=== PickAndMove ready ==="));
  Serial.println(F("Commands: PIVOT_LEFT/PIVOT_RIGHT/DRIVE_FWD/DRIVE_BACK <speed>, STOP"));
}

// ── Low level motor helpers ─────────────────────────────────────────────────
void setMotorLeft(int dir, int speed) {
  if (dir > 0)      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, speed); }
  else if (dir < 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, speed); }
  else              { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  analogWrite(ENA, 0); }
}

void setMotorRight(int dir, int speed) {
  if (dir > 0)      { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, speed); }
  else if (dir < 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, speed); }
  else              { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  analogWrite(ENB, 0); }
}

void stopMotors() {
  setMotorLeft(0, 0);
  setMotorRight(0, 0);
  stopAt = 0;
}

// Start a timed motion pulse.
void startPulse() {
  stopAt = millis() + PULSE_MS;
}

// ── High level moves ─────────────────────────────────────────────────────────
void pivotLeft(int speed)  { setMotorLeft(-1, speed); setMotorRight( 1, speed); startPulse(); }
void pivotRight(int speed) { setMotorLeft( 1, speed); setMotorRight(-1, speed); startPulse(); }
void driveForward(int speed) { setMotorLeft( 1, speed); setMotorRight( 1, speed); startPulse(); }
void driveBackward(int speed){ setMotorLeft(-1, speed); setMotorRight(-1, speed); startPulse(); }

// ── Serial parsing ───────────────────────────────────────────────────────────
int parseSpeed(const String &input, int prefixLen) {
  int speed = input.substring(prefixLen).toInt();
  return constrain(speed, 0, 255);
}

void handleSerial() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  if (input.equalsIgnoreCase("STOP")) {
    stopMotors();
    Serial.println(F("STOP"));
  }
  else if (input.startsWith("PIVOT_LEFT")) {
    int s = parseSpeed(input, 10);
    pivotLeft(s);
    Serial.print(F("PIVOT_LEFT "));  Serial.println(s);
  }
  else if (input.startsWith("PIVOT_RIGHT")) {
    int s = parseSpeed(input, 11);
    pivotRight(s);
    Serial.print(F("PIVOT_RIGHT ")); Serial.println(s);
  }
  else if (input.startsWith("DRIVE_FWD")) {
    int s = parseSpeed(input, 9);
    driveForward(s);
    Serial.print(F("DRIVE_FWD "));   Serial.println(s);
  }
  else if (input.startsWith("DRIVE_BACK")) {
    int s = parseSpeed(input, 10);
    driveBackward(s);
    Serial.print(F("DRIVE_BACK "));  Serial.println(s);
  }
  else {
    Serial.print(F("Unknown command: "));
    Serial.println(input);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  handleSerial();

  // Auto-stop when the current pulse expires.
  if (stopAt != 0 && millis() >= stopAt) {
    stopMotors();
  }
}
```

</div>
</details>

---

## `ServoTest.ino`

Minimal sketch to sweep two servos through their range. Used to verify servo wiring, confirm PWM pin assignments, and check for mechanical binding in the 3D-printed joints.

<details>
<summary>Arduino C | 79 lines</summary>
<div class="code-scroll">

```cpp
#include <Servo.h>

Servo baseServo;
Servo shoulderServo;

const int BASE_PIN     = 10;
const int SHOULDER_PIN = 9;

int baseAngle     = 135;
int shoulderAngle = 135;

void setup() {
  Serial.begin(9600);

  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);

  baseServo.write(baseAngle);
  shoulderServo.write(shoulderAngle);

  Serial.println("=== 2-DOF Servo Controller ===");
  Serial.println("Format:  <servo> <angle>");
  Serial.println("  servo : 1=Base  2=Shoulder");
  Serial.println("  angle : 0 - 270");
  Serial.println("Example: 2 45   (moves Shoulder to 45 deg)");
  Serial.println("Type 'status' to see current angles.");
  Serial.println("==============================\n");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("status")) {
      printStatus();
      return;
    }

    int spaceIndex = input.indexOf(' ');
    if (spaceIndex == -1) {
      Serial.println("Invalid format. Use: <servo> <angle>");
      return;
    }

    int servoNum = input.substring(0, spaceIndex).toInt();
    int angle    = input.substring(spaceIndex + 1).toInt();

    if (servoNum < 1 || servoNum > 2) {
      Serial.println("Servo must be 1 or 2.");
      return;
    }
    if (angle < 0 || angle > 270) {
      Serial.println("Angle must be 0-270.");
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
    }
    Serial.print(angle);
    Serial.println(" deg");
  }
}

void printStatus() {
  Serial.println("----- Current Angles -----");
  Serial.print("  1) Base     : "); Serial.println(baseAngle);
  Serial.print("  2) Shoulder : "); Serial.println(shoulderAngle);
  Serial.println("--------------------------");
}
```

</div>
</details>

---

## `SesnsorTest.ino`

IMU sensor test. Reads raw accelerometer and gyroscope values from the MPU6050 and prints them to the Serial Monitor. Used to verify I²C communication and check sensor orientation before writing the pitch calculation.

<details>
<summary>Arduino C | 66 lines</summary>
<div class="code-scroll">

```cpp
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
```

</div>
</details>

---

## `PumpCheck.ino`

Minimal pump relay test. Toggles the vacuum pump on and off via serial commands with no other hardware active. Used to isolate and debug pump-related brownout issues before the two-rail power redesign.

<details>
<summary>Arduino C | 52 lines</summary>
<div class="code-scroll">

```cpp
// PickMasters — PumpCheck
// Minimal firmware to verify the gamepad controller toggles the pump over USB
// serial. No servos, motors, or IMU — so nothing can stall boot or brown out.
//
// Pin:
//   3  — Pump relay (ACTIVE LOW: LOW = on, HIGH = off, starts off)
//
// Serial (9600 baud, newline-terminated):
//   P1      — pump ON
//   P0      — pump OFF
//   status  — report current pump state

#define PUMP_PIN 3

bool pumpOn = false;
char buf[16];
byte len = 0;

void setup() {
  Serial.begin(9600);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, HIGH);   // active LOW -> OFF at startup
  Serial.write("PickMasters pump test ready\n");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      buf[len] = '\0';
      handle(buf);
      len = 0;
    } else if (c != '\r' && len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

void handle(char *cmd) {
  if (strcmp(cmd, "P1") == 0) {
    pumpOn = true;
    digitalWrite(PUMP_PIN, LOW);    // active LOW = ON
    Serial.write("pump ON\n");
  } else if (strcmp(cmd, "P0") == 0) {
    pumpOn = false;
    digitalWrite(PUMP_PIN, HIGH);   // active LOW = OFF
    Serial.write("pump OFF\n");
  } else if (strcmp(cmd, "status") == 0) {
    Serial.print("PUMP:");
    Serial.println(pumpOn ? "ON" : "OFF");
  }
}
```

</div>
</details>

---

## `PumpTest.ino`

Bare-minimum relay toggle. Even simpler than PumpCheck — just turns the relay on for a few seconds and off again. Used for initial hardware verification of the relay module wiring.

<details>
<summary>Arduino C | 27 lines</summary>
<div class="code-scroll">

```cpp
const int RELAY_PIN = 7;

void setup() {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Pump off

  Serial.println("=== Pump Relay Test ===");
  Serial.println("'on'  - Turn pump on");
  Serial.println("'off' - Turn pump off");
  Serial.println("=======================\n");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("on")) {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Pump ON");
    }
    else if (input.equalsIgnoreCase("off")) {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Pump OFF");
    }
  }
}
```

</div>
</details>

---

## `Full.ino`

An early Processing sketch that combined gamepad input with basic serial output. Uses the Firmata protocol to drive servos directly. Predates the separation into DriveControl.pde (manual) and detect_and_move.py (auto).

<details>
<summary>Processing (Java) | 89 lines</summary>
<div class="code-scroll">

```java
import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;
import processing.serial.*;
import net.java.games.input.*;
import cc.arduino.*;
import org.firmata.*;

ControlDevice cont;
ControlIO control;
Arduino arduino;

float baseAngle     = 90;
float shoulderAngle = 90;
float elbowAngle    = 90;
float wristAngle    = 90;

float speed = 4; // degrees per frame — adjust to taste
float deadzone = 0.2; // ignore small stick drift

void setup() {
  size(360, 200);
  frameRate(50);

  control = ControlIO.getInstance(this);
  cont = control.getMatchedDevice("Test4");

  if (cont == null) {
    println("Not connected");
    System.exit(-1);
  }

  arduino = new Arduino(this, Arduino.list()[1], 57600);
  arduino.pinMode(9, Arduino.SERVO);
  arduino.pinMode(10, Arduino.SERVO);
  arduino.pinMode(11, Arduino.SERVO);
  arduino.pinMode(12, Arduino.SERVO);
}

public void getUserInput() {
  float baseInput     = cont.getSlider("ServoBase").getValue();
  float shoulderInput = cont.getSlider("ServoShoulder").getValue();
  float elbowInput    = cont.getSlider("ServoElbow").getValue();
  float wristInput    = cont.getSlider("ServoWrist").getValue();

  // Apply deadzone
  if (abs(baseInput) > deadzone)     baseAngle     += baseInput * speed;
  if (abs(shoulderInput) > deadzone) shoulderAngle += shoulderInput * speed;
  if (abs(elbowInput) > deadzone)    elbowAngle    += elbowInput * speed;
  if (abs(wristInput) > deadzone)    wristAngle    += wristInput * speed;

  // Clamp to 0-180
  baseAngle     = constrain(baseAngle, 0, 180);
  shoulderAngle = constrain(shoulderAngle, 0, 180);
  elbowAngle    = constrain(elbowAngle, 0, 180);
  wristAngle    = constrain(wristAngle, 0, 180);

  println("Base: " + baseAngle + "  Shoulder: " + shoulderAngle +
          "  Elbow: " + elbowAngle + "  Wrist: " + wristAngle);
}

void draw() {
  getUserInput();
  background(baseAngle, shoulderAngle, 255);

  arduino.servoWrite(9, (int)baseAngle);
  arduino.servoWrite(10, (int)shoulderAngle);
  arduino.servoWrite(11, (int)elbowAngle);
  arduino.servoWrite(12, (int)wristAngle);
}
```

</div>
</details>

---

## `DriveControl.pde`

Development version of the manual-control gamepad interface. Full button mapping with D-pad, pump toggle, IMU calibration, and wrist AUTO/MANUAL mode switching. Functionally identical to the final version.

<details>
<summary>Processing (Java) | 231 lines</summary>
<div class="code-scroll">

```java
// PickMasters — DriveControl (manual controller, paired with ArmController.ino)
//
// Bluetooth XINPUT gamepad via GameControlPlus (config: data/PickMasters).
//
// Mapping:
//   Left stick (digital)  — base drive: fully forward/back = both wheels,
//                           fully left/right = pivot in place. Constant speed.
//   D-pad up/down         — shoulder servo  (±step per frame while held)
//   D-pad left/right      — elbow servo     (±step per frame while held)
//   Right stick Y         — wrist servo  (digital: full push only, MANUAL mode)
//   Right stick X         — hand servo   (digital: full push only)
//   PumpButton            — pump on/off toggle        (rising edge)
//   CalButton             — IMU re-zero ("cal")       (rising edge)
//   WristModeButton       — wrist AUTO/MANUAL toggle  (rising edge)
//
// Serial: ONE combined message per frame, sent only when something changed
// and at most every SEND_INTERVAL ms, to avoid flooding the Arduino:
//   S<shoulder>,<elbow>,<wrist>,<hand>,M<left>,<right>,P<0|1>,W<0|1>\n

import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;
import g4p_controls.*;
import processing.serial.*;
import net.java.games.input.*;

ControlDevice cont;
ControlIO control;
Serial port;
boolean controllerReady = false;

// --- Servo state ---
float shoulderAngle = 90;
float elbowAngle    = 90;
float wristAngle    = 90;
float handAngle     = 90;

float dpadStep  = 3.0;   // deg per frame while D-pad held (shoulder/elbow)
float stickStep = 3.0;   // deg per frame while right stick fully pushed (wrist/hand)

// --- Drive state (digital: stick must be fully pushed) ---
float driveThreshold = 0.9;
int motorLeft  = 0;      // -1 / 0 / +1, Arduino applies its constant speed
int motorRight = 0;

boolean pumpOn    = false;
boolean wristAuto = true;   // true = IMU leveling, false = right stick Y

// --- Button edge detection ---
boolean prevPumpBtn = false;
boolean prevCalBtn  = false;
boolean prevModeBtn = false;

// --- Serial throttle: send only on change, at most every SEND_INTERVAL ms ---
String lastMsg  = "";
long   lastSend = 0;
final int SEND_INTERVAL = 50;

String lastResponse = "";

void setup() {
  size(440, 290);
  frameRate(50);

  try {
    control = ControlIO.getInstance(this);
    cont = control.getMatchedDevice("PickMasters");
  } catch (Exception e) {
    println("Warning during controller init: " + e.getMessage());
  }

  if (cont == null) {
    println("Controller not found — check data/PickMasters config");
    System.exit(-1);
  }
  controllerReady = true;

  String[] ports = Serial.list();
  printArray(ports);

  String portName = null;
  for (int i = ports.length - 1; i >= 0; i--) {
    if (!ports[i].equals("COM1")) { portName = ports[i]; break; }
  }
  if (portName == null && ports.length > 0) portName = ports[0];

  if (portName == null) {
    println("No serial port found — is the Arduino plugged in?");
    System.exit(-1);
  }

  println("Connecting to " + portName);
  port = new Serial(this, portName, 9600);
  port.bufferUntil('\n');

  delay(2000);   // let the Arduino reboot after the port opens
}

void getUserInput() {
  if (!controllerReady || cont == null) return;
  float leftX  = cont.getSlider("LeftX").getValue();
  float leftY  = cont.getSlider("LeftY").getValue();
  float rightX = cont.getSlider("RightX").getValue();
  float rightY = cont.getSlider("RightY").getValue();

  if (leftY <= -driveThreshold)      { motorLeft =  1; motorRight =  1; }  // forward
  else if (leftY >= driveThreshold)  { motorLeft = -1; motorRight = -1; }  // backward
  else if (leftX >= driveThreshold)  { motorLeft =  1; motorRight = -1; }  // pivot right
  else if (leftX <= -driveThreshold) { motorLeft = -1; motorRight =  1; }  // pivot left
  else                               { motorLeft =  0; motorRight =  0; }

  int pos = cont.getHat("Dpad").getPos();
  boolean dUp    = (pos == 1 || pos == 2 || pos == 3);
  boolean dDown  = (pos == 5 || pos == 6 || pos == 7);
  boolean dRight = (pos == 3 || pos == 4 || pos == 5);
  boolean dLeft  = (pos == 7 || pos == 8 || pos == 1);

  if (dUp)    shoulderAngle += dpadStep;
  if (dDown)  shoulderAngle -= dpadStep;
  if (dRight) elbowAngle    += dpadStep;
  if (dLeft)  elbowAngle    -= dpadStep;

  if (!wristAuto) {
    if (rightY <= -driveThreshold)     wristAngle += stickStep;
    else if (rightY >= driveThreshold) wristAngle -= stickStep;
  }
  if (rightX >= driveThreshold)        handAngle += stickStep;
  else if (rightX <= -driveThreshold)  handAngle -= stickStep;

  shoulderAngle = constrain(shoulderAngle, 0, 180);
  elbowAngle    = constrain(elbowAngle,    0, 180);
  wristAngle    = constrain(wristAngle,    0, 180);
  handAngle     = constrain(handAngle,     0, 180);

  boolean pumpBtn = cont.getButton("PumpButton").pressed();
  boolean calBtn  = cont.getButton("CalButton").pressed();
  boolean modeBtn = cont.getButton("WristModeButton").pressed();

  if (pumpBtn && !prevPumpBtn) pumpOn = !pumpOn;
  if (modeBtn && !prevModeBtn) wristAuto = !wristAuto;
  if (calBtn && !prevCalBtn)   port.write("cal\n");

  prevPumpBtn = pumpBtn;
  prevCalBtn  = calBtn;
  prevModeBtn = modeBtn;
}

void sendState() {
  String msg = "S" + (int)shoulderAngle + "," + (int)elbowAngle + ","
                   + (int)wristAngle + "," + (int)handAngle
             + ",M" + motorLeft + "," + motorRight
             + ",P" + (pumpOn ? 1 : 0)
             + ",W" + (wristAuto ? 1 : 0) + "\n";

  if (!msg.equals(lastMsg) && millis() - lastSend >= SEND_INTERVAL) {
    port.write(msg);
    lastMsg  = msg;
    lastSend = millis();
  }
}

void draw() {
  try {
    getUserInput();
  } catch (java.util.ConcurrentModificationException e) {
    // skip this frame's input, will re-read next frame
  }
  sendState();

  background(40, 60, 100);

  fill(255);
  textSize(16);
  text("PickMasters  —  Manual Mode", 10, 28);

  textSize(14);
  fill(200, 230, 255);
  text("Shoulder:  " + (int)shoulderAngle + " deg", 10, 60);
  text("Elbow:     " + (int)elbowAngle + " deg", 10, 82);
  text("Wrist:     " + (int)wristAngle + " deg", 10, 104);
  text("Hand:      " + (int)handAngle + " deg", 10, 126);
  text("Motors:    L " + motorState(motorLeft) + "   R " + motorState(motorRight), 10, 148);

  fill(pumpOn ? color(80, 255, 80) : color(255, 80, 80));
  text("Pump:      " + (pumpOn ? "ON" : "OFF"), 10, 170);

  fill(wristAuto ? color(120, 200, 255) : color(255, 200, 80));
  text("Wrist mode: " + (wristAuto ? "AUTO (IMU)" : "MANUAL (right stick Y)"), 10, 192);

  fill(160);
  textSize(11);
  text("Left stick: drive (full push)   D-pad: shoulder/elbow   Right stick: wrist/hand", 10, 230);
  text("PumpButton: pump   CalButton: IMU re-zero   WristModeButton: AUTO/MANUAL", 10, 247);
  fill(220);
  text("Arduino: " + lastResponse, 10, 275);
}

String motorState(int dir) {
  if (dir > 0) return "FWD";
  if (dir < 0) return "REV";
  return "STOP";
}

void serialEvent(Serial p) {
  String msg = p.readStringUntil('\n');
  if (msg != null) {
    lastResponse = msg.trim();
    println(lastResponse);
  }
}
```

</div>
</details>

---

## `ManualMode.pde`

An earlier, simpler Processing sketch for manual control. Three-axis gamepad mapping with no serial throttling — replaced by the full DriveControl once the control mapping was finalized.

<details>
<summary>Processing (Java) | 90 lines</summary>
<div class="code-scroll">

```java
import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;
import processing.serial.*;
import net.java.games.input.*;

ControlDevice cont;
ControlIO control;
Serial port;

float baseAngle     = 90;
float shoulderAngle = 90;
float elbowAngle    = 90;

float speed    = 4.0;
float deadzone = 0.2;

// Track previous angles to only send when changed
int prevBase     = 90;
int prevShoulder = 90;
int prevElbow    = 90;

void setup() {
  size(360, 200);
  frameRate(50);

  control = ControlIO.getInstance(this);
  cont = control.getMatchedDevice("Test4");

  if (cont == null) {
    println("Controller not connected");
    System.exit(-1);
  }

  println(Serial.list());
  port = new Serial(this, Serial.list()[1], 9600);
  port.bufferUntil('\n');

  delay(2000); // Wait for Arduino to boot
}

public void getUserInput() {
  float baseInput     = cont.getSlider("ServoBase").getValue();
  float shoulderInput = cont.getSlider("ServoShoulder").getValue();
  float elbowInput    = cont.getSlider("ServoElbow").getValue();

  if (abs(baseInput) > deadzone)     baseAngle     += baseInput * speed;
  if (abs(shoulderInput) > deadzone) shoulderAngle += shoulderInput * speed;
  if (abs(elbowInput) > deadzone)    elbowAngle    += elbowInput * speed;

  baseAngle     = constrain(baseAngle, 0, 180);
  shoulderAngle = constrain(shoulderAngle, 0, 180);
  elbowAngle    = constrain(elbowAngle, 0, 180);
}

void sendCommand(int servo, int angle) {
  port.write(servo + " " + angle + "\n");
}

void draw() {
  getUserInput();
  background(baseAngle, shoulderAngle, 255);

  // Only send when angle actually changes
  if ((int)baseAngle != prevBase) {
    sendCommand(1, (int)baseAngle);
    prevBase = (int)baseAngle;
  }
  if ((int)shoulderAngle != prevShoulder) {
    sendCommand(2, (int)shoulderAngle);
    prevShoulder = (int)shoulderAngle;
  }
  if ((int)elbowAngle != prevElbow) {
    sendCommand(3, (int)elbowAngle);
    prevElbow = (int)elbowAngle;
  }

  // Display on screen
  fill(0);
  textSize(14);
  text("Base: " + (int)baseAngle, 10, 30);
  text("Shoulder: " + (int)shoulderAngle, 10, 50);
  text("Elbow: " + (int)elbowAngle, 10, 70);
  text("Wrist: AUTO (IMU)", 10, 90);
}

// Print any feedback from Arduino
void serialEvent(Serial p) {
  String msg = p.readStringUntil('\n');
  if (msg != null) println(msg.trim());
}
```

</div>
</details>

---

## `PumpCheckDrive.pde`

Processing companion for `PumpCheck.ino`. Adds gamepad-based pump toggle to verify the full input chain: gamepad → Processing → serial → Arduino → relay → pump.

<details>
<summary>Processing (Java) | 99 lines</summary>
<div class="code-scroll">

```java
// PickMasters — PumpCheckDrive (controller test, paired with PumpCheck.ino)
//
// Minimal Processing sketch: reads ONLY the pump button on the Bluetooth
// gamepad (GameControlPlus, config: data/PickMasters) and toggles the pump on
// the Arduino. Use this to confirm the whole chain works:
//   gamepad button -> serial -> Arduino -> pump relay.
//
// Serial: "P1\n" = pump on, "P0\n" = pump off (9600 baud).

import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;
import g4p_controls.*;
import processing.serial.*;
import net.java.games.input.*;

ControlDevice cont;
ControlIO control;
Serial port;
boolean controllerReady = false;

boolean pumpOn      = false;
boolean prevPumpBtn = false;
String  lastResponse = "";

void setup() {
  size(380, 170);
  frameRate(50);

  // Match the controller (same config file as the full DriveControl sketch).
  try {
    control = ControlIO.getInstance(this);
    cont = control.getMatchedDevice("PickMasters");
  } catch (Exception e) {
    println("Warning during controller init: " + e.getMessage());
  }
  if (cont == null) {
    println("Controller not found — check data/PickMasters config");
    System.exit(-1);
  }
  controllerReady = true;

  // Pick the Arduino's serial port: prefer the last port that isn't COM1.
  String[] ports = Serial.list();
  printArray(ports);

  String portName = null;
  for (int i = ports.length - 1; i >= 0; i--) {
    if (!ports[i].equals("COM1")) { portName = ports[i]; break; }
  }
  if (portName == null && ports.length > 0) portName = ports[0];
  if (portName == null) {
    println("No serial port found — is the Arduino plugged in?");
    System.exit(-1);
  }

  println("Connecting to " + portName);
  port = new Serial(this, portName, 9600);
  port.bufferUntil('\n');
  delay(2000);   // let the Arduino reboot after the port opens
}

void draw() {
  // ConcurrentModificationException is a known GCP bug — skip the frame.
  try {
    if (controllerReady && cont != null) {
      boolean pumpBtn = cont.getButton("PumpButton").pressed();
      if (pumpBtn && !prevPumpBtn) {          // rising edge = press
        pumpOn = !pumpOn;
        port.write(pumpOn ? "P1\n" : "P0\n");
      }
      prevPumpBtn = pumpBtn;
    }
  } catch (java.util.ConcurrentModificationException e) {
    // skip this frame's input
  }

  background(40, 60, 100);
  fill(255);
  textSize(16);
  text("PickMasters  —  Pump Test", 10, 28);

  fill(pumpOn ? color(80, 255, 80) : color(255, 80, 80));
  textSize(22);
  text("PUMP: " + (pumpOn ? "ON" : "OFF"), 10, 72);

  fill(200);
  textSize(12);
  text("Press the PumpButton on the controller to toggle.", 10, 105);
  fill(220);
  text("Arduino: " + lastResponse, 10, 140);
}

void serialEvent(Serial p) {
  String msg = p.readStringUntil('\n');
  if (msg != null) {
    lastResponse = msg.trim();
    println(lastResponse);
  }
}
```

</div>
</details>

---

## `School.pde`

Minimal Processing test sketch used during early lab sessions. Basic gamepad-to-servo serial communication using the Firmata protocol.

<details>
<summary>Processing (Java) | 69 lines</summary>
<div class="code-scroll">

```java
import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;
import processing.serial.*;
import net.java.games.input.*;

ControlDevice cont;
ControlIO control;
Serial port;

float baseAngle     = 90;
float shoulderAngle = 90;
float elbowAngle    = 90;

float speed    = 2.0;
float deadzone = 0.2;

// Track previous angles to only send when changed
int prevBase     = 90;
int prevShoulder = 90;
int prevElbow    = 90;

void setup() {
  size(360, 200);
  frameRate(50);

  control = ControlIO.getInstance(this);
  cont = control.getMatchedDevice("Test4");

  if (cont == null) {
    println("Controller not connected");
    System.exit(-1);
  }

  println(Serial.list());
  port = new Serial(this, Serial.list()[1], 9600);
  port.bufferUntil('\n');

  delay(2000); // Wait for Arduino to boot
}

public void getUserInput() {
  float baseInput     = cont.getSlider("ServoBase").getValue();
  float shoulderInput = cont.getSlider("ServoShoulder").getValue();
  float elbowInput    = cont.getSlider("ServoElbow").getValue();

  if (abs(baseInput) > deadzone)     baseAngle     += baseInput * speed;
  if (abs(shoulderInput) > deadzone) shoulderAngle += shoulderInput * speed;
  if (abs(elbowInput) > deadzone)    elbowAngle    += elbowInput * speed;

  baseAngle     = constrain(baseAngle, 0, 180);
  shoulderAngle = constrain(shoulderAngle, 0, 180);
  elbowAngle    = constrain(elbowAngle, 0, 180);
}

void sendCommand(int servo, int angle) {
  port.write(servo + " " + angle + "\n");
}

void draw() {
  getUserInput();
  background(baseAngle, shoulderAngle, 255);

  // Only send when angle actually changes
  if ((int)baseAngle != prevBase) {
    sendCommand(1, (int)baseAngle);
    prevBase = (int)baseAngle;
  }
  if ((int)shoulderAngle != prevShoulder) {
    sendCommand(2, (int)shoulderAngle);
    prevShoulder = (int)shoulderAngle;
  }
  if ((int)elbowAngle != prevElbow) {
    sendCommand(3, (int)elbowAngle);
    prevElbow = (int)elbowAngle;
  }

  // Display on screen
  fill(0);
  textSize(14);
  text("Base: " + (int)baseAngle, 10, 30);
  text("Shoulder: " + (int)shoulderAngle, 10, 50);
  text("Elbow: " + (int)elbowAngle, 10, 70);
  text("Wrist: AUTO (IMU)", 10, 90);
}

// Print any feedback from Arduino
void serialEvent(Serial p) {
  String msg = p.readStringUntil('\n');
  if (msg != null) println(msg.trim());
}
```

</div>
</details>

---

[← Back to Home](../index.md)
