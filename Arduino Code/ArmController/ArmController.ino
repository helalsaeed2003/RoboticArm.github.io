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
