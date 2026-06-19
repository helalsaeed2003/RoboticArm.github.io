// PickMasters — ArmController firmware
// 4-DOF robotic arm, manual control over USB serial (paired with DriveControl.pde)
//
// Hardware: Arduino Uno + Adafruit L293D Motor Shield v1 (AFMotor library)
//
// Pin usage:
//   A0  — Shoulder servo
//   A1  — Elbow servo
//   A2  — Wrist servo
//   A3  — Hand servo
//   A4  — IMU SDA (I2C, reserved by Wire)
//   A5  — IMU SCL (I2C, reserved by Wire)
//   2   — Pump relay (ACTIVE LOW: LOW = on, HIGH = off, starts off)
//   9   — IMU calibration button (INPUT_PULLUP)
//   Motor shield occupies pins 3,4,5,6,7,8,11,12 internally — do not reuse.
//   DC motors: M1 = left wheel, M2 = right wheel.
//
// Serial protocol (9600 baud, newline-terminated, ONE combined message per frame):
//   S<shoulder>,<elbow>,<wrist>,<hand>,M<left>,<right>,P<0|1>,W<0|1>
//     servo angles 0..180, motor directions -1/0/1, P1 = pump on,
//     W1 = wrist AUTO (IMU leveling), W0 = wrist MANUAL
//   cal     — re-zero IMU pitch offset (also hardware button on pin 9)
//   status  — print all current angles and states (single compact line)

#include <Wire.h>
#include <Servo.h>
#include <AFMotor.h>   // Adafruit Motor Shield v1 library

// ── Pin definitions ──────────────────────────────────────────────────────────
#define SHOULDER_PIN  A0
#define ELBOW_PIN     A1
#define WRIST_PIN     A2
#define HAND_PIN      A3
#define PUMP_PIN      2     // relay is active LOW
#define CAL_BUTTON    9     // INPUT_PULLUP — press to re-zero IMU
#define MPU_ADDR      0x68

// ── DC motors via L293D motor shield ─────────────────────────────────────────
AF_DCMotor motorLeft(1);
AF_DCMotor motorRight(2);
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

  // Pump relay — active LOW, so HIGH = OFF at startup
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, HIGH);

  pinMode(CAL_BUTTON, INPUT_PULLUP);

  // Servos — centre on startup
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  wristServo.attach(WRIST_PIN);
  handServo.attach(HAND_PIN);
  shoulderServo.write(shoulderAngle);
  elbowServo.write(elbowAngle);
  wristServo.write(wristAngle);
  handServo.write(handAngle);

  // Motors idle
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);

  // Wake up MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0x00);   // clear sleep bit
  Wire.endTransmission(true);
  delay(200);

  calibrateIMU();
  Serial.write("PickMasters ready\n");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  handleSerial();
  handleCalButton();

  if (millis() - lastIMU >= 20) {   // 50 Hz IMU / wrist update
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

      setMotor(motorLeft,  l);
      setMotor(motorRight, r);
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

// ── DC motor control ──────────────────────────────────────────────────────────
void setMotor(AF_DCMotor &m, int dir) {
  if (dir > 0) {
    m.setSpeed(DRIVE_SPEED);
    m.run(FORWARD);
  } else if (dir < 0) {
    m.setSpeed(DRIVE_SPEED);
    m.run(BACKWARD);
  } else {
    m.setSpeed(0);
    m.run(RELEASE);
  }
}

// ── IMU wrist auto-level ──────────────────────────────────────────────────────
float readPitch() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // ACCEL_XOUT_H — start of 6-byte accel block
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

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
  Serial.print(" PITCH:"); Serial.print(currentPitch - pitchOffset, 1);
  Serial.write('\n');
}
