# Final Code

[← Back to Home](../index.md)

---

These are the four production programs that run the PickMasters system. Together they give the robot both manual and automatic control. The Arduino sketches handle real-time motor and servo control; the host programs handle vision and human input.

---

## ArmController.ino — Manual Mode Firmware

> Arduino C | 379 lines | Runs on Arduino Uno R3

The complete manual-mode firmware. Handles all four arm servos, both DC base motors via L298N, the vacuum pump relay, IMU-based wrist auto-levelling (PID), a three-condition safety interlock, XOR-checksum serial protocol, and a 2-second hardware watchdog.

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
//   A0  — Safety interlock input (HIGH = clear, active-high with pull-down)
//   L298N driver: ENA=5, IN1=2, IN2=4 (left wheel);
//                 ENB=6, IN3=7, IN4=8 (right wheel).
//
// Serial protocol (9600 baud, newline-terminated, ONE combined message per frame):
//   S<shoulder>,<elbow>,<wrist>,<hand>,M<left>,<right>,P<0|1>,W<0|1>*<checksum>
//     servo angles 0..180, motor directions -1/0/1, P1 = pump on,
//     W1 = wrist AUTO (IMU leveling), W0 = wrist MANUAL
//     *<checksum> = optional XOR checksum (two hex digits) of all bytes before '*'
//   cal     — re-zero IMU pitch offset (sent by the DriveControl CalButton)
//   status  — print all current angles and states (single compact line)

#include <Wire.h>
#include <Servo.h>
#include <avr/wdt.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
#define SHOULDER_PIN  9
#define ELBOW_PIN     10
#define WRIST_PIN     11
#define HAND_PIN      12
#define PUMP_PIN      3     // relay is active LOW
#define MPU_ADDR      0x68
#define SAFETY_PIN    A0    // safety interlock input

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

// ── PID state for wrist auto-level ───────────────────────────────────────────
float pidKp = 1.0;
float pidKi = 0.05;
float pidKd = 0.15;
float pidIntegral   = 0.0;
float pidLastError  = 0.0;
unsigned long pidLastTime = 0;
const float PID_INTEGRAL_LIMIT = 30.0;

// ── Safety interlock ─────────────────────────────────────────────────────────
bool safetyClear = false;
unsigned long lastSafetyCheck = 0;
const unsigned long SAFETY_CHECK_MS = 50;
unsigned long lastSerialRx = 0;
const unsigned long SERIAL_TIMEOUT_MS = 2000;

// ── Serial receive buffer ─────────────────────────────────────────────────────
char serialBuf[64];
byte serialLen = 0;

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Serial.write("PickMasters booting\n");

  // Safety interlock input (active-high with external pull-down resistor)
  pinMode(SAFETY_PIN, INPUT);

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

  // I2C — a timeout so a missing/shorted IMU can never freeze the board.
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

  lastSerialRx = millis();
  pidLastTime  = millis();

  // Hardware watchdog: resets the MCU if loop() hangs for >2 seconds
  wdt_enable(WDTO_2S);

  Serial.write("PickMasters ready\n");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  wdt_reset();

  handleSerial();
  updateSafetyInterlock();

  if (imuOk && millis() - lastIMU >= 20) {   // 50 Hz IMU / wrist update
    lastIMU = millis();
    currentPitch = readPitch();
    if (wristAutoMode) {
      wristAngle = pidCompute(currentPitch - pitchOffset);
      wristServo.write(wristAngle);
    }
  }
}

// ── PID controller for wrist auto-level ──────────────────────────────────────
int pidCompute(float error) {
  unsigned long now = millis();
  float dt = (now - pidLastTime) / 1000.0;
  if (dt <= 0) dt = 0.02;
  pidLastTime = now;

  pidIntegral += error * dt;
  pidIntegral = constrain(pidIntegral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

  float derivative = (error - pidLastError) / dt;
  pidLastError = error;

  float output = 90.0 + (pidKp * error) + (pidKi * pidIntegral) + (pidKd * derivative);
  return constrain((int)output, 0, 180);
}

// ── Safety interlock ─────────────────────────────────────────────────────────
// Movement is blocked unless ALL conditions are met:
//   1. SAFETY_PIN reads HIGH (hardware interlock / e-stop circuit)
//   2. Serial link is alive (received a command within SERIAL_TIMEOUT_MS)
//   3. IMU is healthy OR wrist is in manual mode
void updateSafetyInterlock() {
  if (millis() - lastSafetyCheck < SAFETY_CHECK_MS) return;
  lastSafetyCheck = millis();

  bool hwClear     = (digitalRead(SAFETY_PIN) == HIGH);
  bool serialAlive = (millis() - lastSerialRx < SERIAL_TIMEOUT_MS);
  bool sensorOk    = imuOk || !wristAutoMode;

  bool wasClear = safetyClear;
  safetyClear = hwClear && serialAlive && sensorOk;

  if (wasClear && !safetyClear) {
    setMotorLeft(0);
    setMotorRight(0);
    digitalWrite(PUMP_PIN, HIGH);
    pumpOn = false;
    Serial.write("SAFETY: interlock OPEN — motors and pump disabled\n");
  }
  if (!wasClear && safetyClear) {
    Serial.write("SAFETY: interlock CLEAR — movement enabled\n");
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

// ── Checksum verification ─────────────────────────────────────────────────────
// If the command contains '*XX' suffix, verify XOR checksum; otherwise accept as-is.
bool verifyChecksum(char *cmd) {
  char *star = strchr(cmd, '*');
  if (!star) return true;   // no checksum present — backward compatible

  byte computed = 0;
  for (char *p = cmd; p < star; p++) computed ^= (byte)*p;

  unsigned int received;
  if (sscanf(star + 1, "%02X", &received) != 1) return false;
  *star = '\0';   // strip checksum from command for parsing

  return (computed == (byte)received);
}

void parseCommand(char *cmd) {
  if (cmd[0] == '\0') return;

  lastSerialRx = millis();

  if (!verifyChecksum(cmd)) {
    Serial.write("ERR: checksum mismatch\n");
    return;
  }

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

      if (safetyClear) {
        setMotorLeft(l);
        setMotorRight(r);
        leftDir  = l;
        rightDir = r;
        pumpOn = (p != 0);
        digitalWrite(PUMP_PIN, pumpOn ? LOW : HIGH);
      } else {
        setMotorLeft(0);
        setMotorRight(0);
        leftDir = 0;
        rightDir = 0;
        pumpOn = false;
        digitalWrite(PUMP_PIN, HIGH);
      }
    }

  } else if (strcmp(cmd, "cal") == 0) {
    calibrateIMU();
    pidIntegral  = 0.0;
    pidLastError = 0.0;
    Serial.write("cal ok\n");

  } else if (strcmp(cmd, "status") == 0) {
    printStatus();

  } else if (strcmp(cmd, "pid") == 0) {
    Serial.print("PID Kp="); Serial.print(pidKp, 2);
    Serial.print(" Ki=");    Serial.print(pidKi, 2);
    Serial.print(" Kd=");    Serial.print(pidKd, 2);
    Serial.write('\n');
  }
}

// ── DC motor control (L298N) ──────────────────────────────────────────────────
// Left wheel: ENA + IN1/IN2.  dir > 0 = forward, dir < 0 = backward, 0 = stop.
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

// Right wheel: ENB + IN3/IN4.  dir > 0 = forward, dir < 0 = backward, 0 = stop.
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
  Wire.write(0x3B);   // ACCEL_XOUT_H — start of 6-byte accel block
  Wire.endTransmission(false);

  // If the IMU doesn't return all 6 bytes (unplugged/glitch), keep the last
  // pitch instead of computing garbage from -1 reads.
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
  Serial.print(" MODE:"); Serial.print(wristAutoMode ? "AUTO/PID" : "MAN");
  Serial.print(" IMU:");  Serial.print(imuOk ? "OK" : "NONE");
  Serial.print(" PITCH:"); Serial.print(currentPitch - pitchOffset, 1);
  Serial.print(" SAFE:");  Serial.print(safetyClear ? "CLEAR" : "OPEN");
  Serial.print(" WDT:ON");
  Serial.write('\n');
}
```

---

## PickAndMove.ino — Automatic Mode Firmware

> Arduino C | 203 lines | Runs on Arduino Uno R3

The automatic-mode firmware. Receives commands from `detect_and_move.py` and drives the DC base motors using a Mamdani fuzzy-logic controller. Motion is pulse-based (70 ms per pulse, then auto-stop) — a dead-man's-switch design where any communication failure halts the robot.

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
//   FUZZY_PIVOT <error>    fuzzy-logic pivot: error = signed pixel offset from centre
//   FUZZY_DRIVE <error>    fuzzy-logic drive: error = signed pixel offset from centre
//   STOP                   stop both motors
//
// Motion is PULSE based: each command runs the motors for PULSE_MS then stops
// automatically.  That way a dropped serial link can never leave the robot
// running away — the Python loop keeps sending pulses while it needs to move.
//
// Fuzzy Logic controller: maps pixel error to motor speed using trapezoidal
// membership functions for {Small, Medium, Large} error and {Slow, Medium, Fast}
// speed output, then defuzzifies via centre-of-gravity.

#include <avr/wdt.h>

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

  wdt_enable(WDTO_2S);

  Serial.println(F("=== PickAndMove ready ==="));
  Serial.println(F("Commands: PIVOT_LEFT/PIVOT_RIGHT/DRIVE_FWD/DRIVE_BACK <speed>,"));
  Serial.println(F("          FUZZY_PIVOT/FUZZY_DRIVE <error>, STOP"));
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

// ── Fuzzy Logic speed controller ──────────────────────────────────────────────
// Membership functions for error magnitude (pixels):
//   Small:  full at 0, drops to 0 at 60
//   Medium: rises from 0 at 30, full at 80-120, drops to 0 at 170
//   Large:  rises from 0 at 120, full at 200+
// Output singletons: Slow=80, Medium=150, Fast=230
//
// Rule base:
//   IF error IS Small  THEN speed IS Slow
//   IF error IS Medium THEN speed IS Medium
//   IF error IS Large  THEN speed IS Fast

float fuzzyTriangle(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0;
  if (x <= b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}

float fuzzyTrapezoid(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0.0;
  if (x >= b && x <= c) return 1.0;
  if (x < b) return (x - a) / (b - a);
  return (d - x) / (d - c);
}

int fuzzyComputeSpeed(int pixelError) {
  float absErr = abs(pixelError);

  float muSmall  = fuzzyTrapezoid(absErr, -1, 0, 20, 60);
  float muMedium = fuzzyTrapezoid(absErr, 30, 80, 120, 170);
  float muLarge  = fuzzyTrapezoid(absErr, 120, 200, 300, 301);

  const float outSlow   = 80.0;
  const float outMedium = 150.0;
  const float outFast   = 230.0;

  float numerator   = muSmall * outSlow + muMedium * outMedium + muLarge * outFast;
  float denominator = muSmall + muMedium + muLarge;

  if (denominator < 0.001) return 60;   // dead zone fallback — minimal creep
  return constrain((int)(numerator / denominator), 0, 255);
}

// ── Serial parsing ───────────────────────────────────────────────────────────
int parseSpeed(const String &input, int prefixLen) {
  int speed = input.substring(prefixLen).toInt();
  return constrain(speed, 0, 255);
}

int parseError(const String &input, int prefixLen) {
  return input.substring(prefixLen).toInt();
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
  else if (input.startsWith("FUZZY_PIVOT")) {
    int err = parseError(input, 11);
    int spd = fuzzyComputeSpeed(err);
    if (err < 0) pivotLeft(spd); else pivotRight(spd);
    Serial.print(F("FUZZY_PIVOT err=")); Serial.print(err);
    Serial.print(F(" spd="));            Serial.println(spd);
  }
  else if (input.startsWith("FUZZY_DRIVE")) {
    int err = parseError(input, 11);
    int spd = fuzzyComputeSpeed(err);
    if (err > 0) driveForward(spd); else driveBackward(spd);
    Serial.print(F("FUZZY_DRIVE err=")); Serial.print(err);
    Serial.print(F(" spd="));            Serial.println(spd);
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
  wdt_reset();
  handleSerial();

  if (stopAt != 0 && millis() >= stopAt) {
    stopMotors();
  }
}
```

---

## detect_and_move.py — Vision and Auto-Control Script

> Python 3 | 260 lines | Runs on host computer | Requires: ultralytics, opencv-python, pyserial

The automatic-mode host script. Loads a YOLOv11n model, opens the USB camera, detects and tracks objects frame-by-frame, locks onto the highest-confidence target, and sends fuzzy-logic motion commands to the Arduino over serial.

```python
"""
detect_and_move.py
==================

Vision + control loop for the PickMasters arm.

The camera feed is split by ONE vertical and ONE horizontal line.  Where the two
lines cross there is a target BOX.  The job of this program is to drive the robot
(DC motors only) until the chosen item sits completely inside that box:

  * VERTICAL line   -> handled by SPINNING the base (pivot the wheels left/right)
                       so the item lines up on the vertical (X) axis.
  * HORIZONTAL line -> handled by MOVING the base forward/back (drive both wheels)
                       so the item lines up on the horizontal (Y) axis.

Locking behaviour
-----------------
The program will only LOCK onto an item once it is seen with >= 80% confidence.
After that it stays focused on the SAME item and keeps centering it, even if the
confidence drops, until you press the "next" key (n).  Then it releases the lock
and is free to pick the next >= 80% item.

Keys
----
  n : release the current lock and look for the next item
  s : send an emergency STOP to the Arduino
  q : quit

Serial protocol (sent to the Arduino, one command per line)
-----------------------------------------------------------
  PIVOT_LEFT  <speed>   spin base left
  PIVOT_RIGHT <speed>   spin base right
  DRIVE_FWD   <speed>   move base forward
  DRIVE_BACK  <speed>   move base backward
  STOP                  stop all DC motors
"""

import os
import cv2
from ultralytics import YOLO
import serial
import time

# --------------------------------------------------------------------------- #
#  Configuration
# --------------------------------------------------------------------------- #
SERIAL_PORT = "COM10"          # change to match your Arduino port
BAUD_RATE   = 9600

# Locate model_v2.pt next to this script so the path works on any machine.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "model_v2.pt")

CAMERA_INDEX = 0

FRAME_WIDTH  = 640
FRAME_HEIGHT = 480
CENTER_X     = FRAME_WIDTH  // 2
# Shift the target (box + horizontal line) lower on the Y axis. Increase
# BOX_Y_OFFSET to push it further down the frame.
BOX_Y_OFFSET = 80
CENTER_Y     = FRAME_HEIGHT // 2 + BOX_Y_OFFSET

# Half-size of the target box drawn at the crossing of the two lines.
# The item must fit COMPLETELY inside this box.
BOX_HALF_W = 55
BOX_HALF_H = 55

BOX_LEFT   = CENTER_X - BOX_HALF_W
BOX_RIGHT  = CENTER_X + BOX_HALF_W
BOX_TOP    = CENTER_Y - BOX_HALF_H
BOX_BOTTOM = CENTER_Y + BOX_HALF_H

# How close (in pixels) the item centre must be to a line before we stop nudging
# on that axis.  Keep this smaller than the box so the item ends up well inside.
DEAD_ZONE_X = 30
DEAD_ZONE_Y = 30

LOCK_CONFIDENCE = 0.50         # must reach this to LOCK onto an item

PIVOT_SPEED = 90               # PWM (0-255) the Arduino uses while spinning base
DRIVE_SPEED = 130              # PWM (0-255) the Arduino uses while driving base

COOLDOWN_MAX = 6               # frames to wait between motion pulses (higher = gentler)


# --------------------------------------------------------------------------- #
#  Setup
# --------------------------------------------------------------------------- #
model = YOLO(MODEL_PATH)
# Use the DirectShow backend on Windows -- the default backend can hang for a
# long time while opening the camera.
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if not cap.isOpened():
    raise RuntimeError(
        f"Could not open camera index {CAMERA_INDEX}. "
        f"Try a different CAMERA_INDEX (0, 1, 2 ...)."
    )

try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("Arduino connected!")
except Exception as e:
    print(f"Arduino not found: {e}")
    arduino = None


def send(command):
    """Send a single line command to the Arduino (and echo to the console)."""
    print(f"Sent: {command}")
    if arduino:
        try:
            arduino.write((command + "\n").encode())
        except Exception as e:
            print(f"Send error: {e}")


# --------------------------------------------------------------------------- #
#  Main loop
# --------------------------------------------------------------------------- #
locked_id = None               # track id of the item we are focused on
cooldown  = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Use tracking so each item keeps a stable id between frames.  This is what
    # lets us stay locked on one item even when its confidence drops.
    results = model.track(frame, persist=True, verbose=False)
    annotated = results[0].plot()

    # ---- Draw the two reference lines and the target box ------------------- #
    cv2.line(annotated, (CENTER_X, 0), (CENTER_X, FRAME_HEIGHT), (255, 0, 0), 2)
    cv2.line(annotated, (0, CENTER_Y), (FRAME_WIDTH, CENTER_Y), (255, 0, 0), 2)
    cv2.rectangle(annotated, (BOX_LEFT, BOX_TOP), (BOX_RIGHT, BOX_BOTTOM),
                  (0, 255, 255), 2)

    boxes = results[0].boxes

    # Build a lookup of the boxes that currently have a track id.
    tracked = {}
    if boxes is not None and boxes.id is not None:
        for i in range(len(boxes)):
            tid = int(boxes.id[i])
            tracked[tid] = boxes[i]

    # ---- Acquire a lock if we don't have one ------------------------------ #
    if locked_id is None or locked_id not in tracked:
        # Look for the highest-confidence item that clears the 80% threshold.
        best_id, best_conf = None, 0.0
        for tid, box in tracked.items():
            conf = float(box.conf[0])
            if conf >= LOCK_CONFIDENCE and conf > best_conf:
                best_id, best_conf = tid, conf

        if best_id is not None:
            locked_id = best_id
            print(f"LOCKED onto item id {locked_id} ({best_conf:.2f})")
        else:
            # Nothing to lock onto -> make sure the robot is stopped.
            if cooldown == 0:
                send("STOP")
                cooldown = COOLDOWN_MAX
            locked_id = None

    # ---- Drive toward the locked item ------------------------------------- #
    if locked_id is not None and locked_id in tracked:
        box = tracked[locked_id]
        class_name = results[0].names[int(box.cls[0])]
        confidence = float(box.conf[0])

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        obj_cx = (x1 + x2) // 2
        obj_cy = (y1 + y2) // 2

        cv2.circle(annotated, (obj_cx, obj_cy), 8, (0, 0, 255), -1)

        # Is the whole item inside the box?
        fully_inside = (x1 >= BOX_LEFT and x2 <= BOX_RIGHT and
                        y1 >= BOX_TOP  and y2 <= BOX_BOTTOM)

        command = None
        if fully_inside:
            # Item is completely inside the box -> we're done. Stop and quit.
            send("STOP")
            status, color = f"DONE - {class_name} in box", (0, 255, 0)
            cv2.putText(annotated, status, (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.imshow("Pick & Move", annotated)
            cv2.waitKey(800)          # show the result briefly
            print(f"Item id {locked_id} ({class_name}) fully inside box. Quitting.")
            break
        elif obj_cx < CENTER_X - DEAD_ZONE_X:
            err_x = obj_cx - CENTER_X
            command = f"FUZZY_PIVOT {err_x}"
            status, color = f"FUZZY PIVOT LEFT (err={err_x})", (0, 255, 255)
        elif obj_cx > CENTER_X + DEAD_ZONE_X:
            err_x = obj_cx - CENTER_X
            command = f"FUZZY_PIVOT {err_x}"
            status, color = f"FUZZY PIVOT RIGHT (err={err_x})", (0, 255, 255)
        elif obj_cy < CENTER_Y - DEAD_ZONE_Y:
            err_y = CENTER_Y - obj_cy
            command = f"FUZZY_DRIVE {err_y}"
            status, color = f"FUZZY DRIVE FWD (err={err_y})", (255, 165, 0)
        elif obj_cy > CENTER_Y + DEAD_ZONE_Y:
            err_y = CENTER_Y - obj_cy
            command = f"FUZZY_DRIVE {err_y}"
            status, color = f"FUZZY DRIVE BACK (err={err_y})", (255, 165, 0)
        else:
            # Centre is on the cross but the box isn't fully contained yet
            # (item bigger than the dead zone) -> nudge it the rest of the way.
            command = None
            status, color = "CENTERING...", (0, 255, 0)

        if cooldown == 0:
            send(command if command else "STOP")
            cooldown = COOLDOWN_MAX

        cv2.putText(annotated, status, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(annotated, f"LOCK id{locked_id} {class_name} {confidence:.2f}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    elif locked_id is not None:
        # We have a lock but lost sight of it this frame -> hold position.
        if cooldown == 0:
            send("STOP")
            cooldown = COOLDOWN_MAX
        cv2.putText(annotated, f"SEARCHING for locked id {locked_id}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    else:
        cv2.putText(annotated, "No >=80% item to lock onto",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    if cooldown > 0:
        cooldown -= 1

    cv2.imshow("Pick & Move", annotated)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('n'):
        # Release the lock so the next >=80% item can be chosen.
        print(f"Released lock on id {locked_id}")
        locked_id = None
        send("STOP")
    elif key == ord('s'):
        send("STOP")

send("STOP")
cap.release()
cv2.destroyAllWindows()
if arduino:
    arduino.close()
```

---

## DriveControl.pde — Manual Gamepad Interface

> Processing (Java) | 231 lines | Runs on host computer | Requires: GameControlPlus library

The manual-mode host application. Reads a Bluetooth XINPUT gamepad, maps axes and buttons to servo angles and motor commands, and sends a single combined serial message per frame (only when values change, throttled to 50 ms minimum interval).

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

  // GCP on Windows enumerates every input device (including virtual ones like
  // FakerInput).  Wrapping init in try/catch lets us recover gracefully.
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

  // Pick the Arduino's serial port automatically: COM1 is almost always the
  // PC's built-in port, so prefer the last port that isn't COM1.  If the
  // Arduino is unplugged (or its driver is missing) no usable port exists.
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

  // --- Base DC motors: digital only, single constant speed ---
  // Stick must be fully pushed (gamepads read negative Y when pushed forward).
  // Forward/back wins; left/right pivots in place (never mixed with fwd/back).
  if (leftY <= -driveThreshold)      { motorLeft =  1; motorRight =  1; }  // forward
  else if (leftY >= driveThreshold)  { motorLeft = -1; motorRight = -1; }  // backward
  else if (leftX >= driveThreshold)  { motorLeft =  1; motorRight = -1; }  // pivot right
  else if (leftX <= -driveThreshold) { motorLeft = -1; motorRight =  1; }  // pivot left
  else                               { motorLeft =  0; motorRight =  0; }

  // --- Shoulder & elbow on the D-pad (fixed step per frame while held) ---
  // GameControlPlus hat positions: 0 = released, then clockwise from
  // 1 = up-left: 2 = up, 3 = up-right, 4 = right, 5 = down-right,
  // 6 = down, 7 = down-left, 8 = left.
  int pos = cont.getHat("Dpad").getPos();
  boolean dUp    = (pos == 1 || pos == 2 || pos == 3);
  boolean dDown  = (pos == 5 || pos == 6 || pos == 7);
  boolean dRight = (pos == 3 || pos == 4 || pos == 5);
  boolean dLeft  = (pos == 7 || pos == 8 || pos == 1);

  if (dUp)    shoulderAngle += dpadStep;
  if (dDown)  shoulderAngle -= dpadStep;
  if (dRight) elbowAngle    += dpadStep;
  if (dLeft)  elbowAngle    -= dpadStep;

  // --- Wrist (right stick Y, MANUAL only) & hand (right stick X): DIGITAL ---
  // Like the drive sticks — the servo only moves at FULL deflection, stepping a
  // fixed amount per frame. No proportional/rate control: half-pushed does nothing.
  if (!wristAuto) {
    if (rightY <= -driveThreshold)     wristAngle += stickStep;   // stick up   = wrist up
    else if (rightY >= driveThreshold) wristAngle -= stickStep;   // stick down = wrist down
  }
  if (rightX >= driveThreshold)        handAngle += stickStep;    // stick right = hand +
  else if (rightX <= -driveThreshold)  handAngle -= stickStep;    // stick left  = hand -

  shoulderAngle = constrain(shoulderAngle, 0, 180);
  elbowAngle    = constrain(elbowAngle,    0, 180);
  wristAngle    = constrain(wristAngle,    0, 180);
  handAngle     = constrain(handAngle,     0, 180);

  // --- Buttons (rising edge only) ---
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
  // ONE combined message per frame — only when it changed, throttled to
  // SEND_INTERVAL, written with port.write() (no println), to keep the
  // Arduino's serial buffer from overflowing and dropping the connection.
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
  // ConcurrentModificationException is a known GCP library bug (device list
  // iterated on two threads simultaneously).  Catching it here lets the sketch
  // keep running instead of crashing — the missed frame is harmless.
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

---

[← Back to Home](../index.md)
