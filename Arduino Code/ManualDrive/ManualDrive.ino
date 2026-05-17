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
// Motor 1 = left wheel, Motor 2 = right wheel.
// If the robot drives backwards when commanded forwards, swap motor wires
// or flip the sign in the Processing sketch (negate leftY/rightY).
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
  Wire.write(0x3B);  // ACCEL_XOUT_H — start of 6-byte accel block
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
  // Positive pitch (arm tilts down) increases wrist angle to compensate.
  // Negate the sign here if leveling goes the wrong direction.
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
