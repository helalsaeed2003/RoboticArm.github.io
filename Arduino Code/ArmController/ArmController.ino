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
//   A0  — IMU calibration button (INPUT_PULLUP)
//   L298N driver: ENA=5, IN1=2, IN2=4 (left wheel);
//                 ENB=6, IN3=7, IN4=8 (right wheel).
//
// Serial protocol (9600 baud, newline-terminated, ONE combined message per frame):
//   S<shoulder>,<elbow>,<wrist>,<hand>,M<left>,<right>,P<0|1>,W<0|1>
//     servo angles 0..180, motor directions -1/0/1, P1 = pump on,
//     W1 = wrist AUTO (IMU leveling), W0 = wrist MANUAL
//   cal     — re-zero IMU pitch offset (also hardware button on pin A0)
//   status  — print all current angles and states (single compact line)

#include <Wire.h>
#include <Servo.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
#define SHOULDER_PIN  9
#define ELBOW_PIN     10
#define WRIST_PIN     11
#define HAND_PIN      12
#define PUMP_PIN      3     // relay is active LOW
#define CAL_BUTTON    A0    // INPUT_PULLUP — press to re-zero IMU
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
bool          prevCalBtn = HIGH;   // HIGH = not pressed (PULLUP)

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

  pinMode(CAL_BUTTON, INPUT_PULLUP);

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
  // setWireTimeout(us, reset_on_timeout): bail out and reset the bus instead
  // of blocking forever when the MPU6050 doesn't respond.
  Wire.begin();
  Wire.setWireTimeout(3000, true);

  // Detect the MPU6050 before using it — endTransmission() == 0 means it ACKed.
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
    // No IMU: keep running everything else, just don't auto-level the wrist.
    wristAutoMode = false;
    Serial.write("WARN: IMU not found — wrist auto-level disabled\n");
  }

  Serial.write("PickMasters ready\n");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  handleSerial();
  handleCalButton();

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
    // Combined frame: S<sh>,<el>,<wr>,<ha>,M<l>,<r>,P<p>,W<w>
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
        // MANUAL — write exactly what Processing sent
        wristAngle = constrain(wr, 0, 180);
        wristServo.write(wristAngle);
      }

      setMotorLeft(l);
      setMotorRight(r);
      leftDir  = l;
      rightDir = r;

      pumpOn = (p != 0);
      digitalWrite(PUMP_PIN, pumpOn ? LOW : HIGH);   // active LOW
    }

  } else if (strcmp(cmd, "cal") == 0) {
    calibrateIMU();
    Serial.write("cal ok\n");

  } else if (strcmp(cmd, "status") == 0) {
    printStatus();
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

// ── Hardware calibration button ───────────────────────────────────────────────
void handleCalButton() {
  bool btn = digitalRead(CAL_BUTTON);
  if (btn == LOW && prevCalBtn == HIGH) {   // falling edge = pressed
    calibrateIMU();
    Serial.write("cal ok\n");
  }
  prevCalBtn = btn;
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
