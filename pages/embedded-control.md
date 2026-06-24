# Embedded Control Logic and Programming

[← Back to Home](../index.md)

---

## Two-Sketch Firmware Architecture

Two Arduino sketches, one per mode. **ArmController.ino** (manual mode) drives all four arm servos, both DC motors, the pump relay, the IMU loop, and the safety interlock. **PickAndMove.ino** (automatic mode) drives only the two DC base motors via the fuzzy-logic controller — arm servos are inactive in auto mode and the operator switches back to manual for the pick step. Two sketches give clear responsibility per file and let the two control laws (PID on the wrist; fuzzy on the base) be explained separately. Mode switching requires reflashing — acceptable for the benchtop demo, would change for industrial use. Pin allocation for L298N, MPU6050, pump relay, and safety input is shared between both sketches.

## Arm Controller Firmware — ArmController.ino

Written in C. Main loop runs at full ATmega328P speed; internal timing gates IMU/wrist updates at 50 Hz. Every pass: refresh watchdog → handle pending serial → update safety interlock → if clear and IMU is due, read IMU and update wrist. Organised into six modules: serial parser with XOR checksum, IMU driver, PID wrist controller, safety-interlock state machine, DC motor driver layer over L298N, watchdog supervisor. Full source: Appendix C.1.

<details>
<summary>ArmController.ino — Arduino C | 379 lines</summary>
<div class="code-scroll">
<pre><code>// PickMasters — ArmController firmware
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
//   S&lt;shoulder&gt;,&lt;elbow&gt;,&lt;wrist&gt;,&lt;hand&gt;,M&lt;left&gt;,&lt;right&gt;,P&lt;0|1&gt;,W&lt;0|1&gt;*&lt;checksum&gt;
//     servo angles 0..180, motor directions -1/0/1, P1 = pump on,
//     W1 = wrist AUTO (IMU leveling), W0 = wrist MANUAL
//     *&lt;checksum&gt; = optional XOR checksum (two hex digits) of all bytes before &#x27;*&#x27;
//   cal     — re-zero IMU pitch offset (sent by the DriveControl CalButton)
//   status  — print all current angles and states (single compact line)

#include &lt;Wire.h&gt;
#include &lt;Servo.h&gt;
#include &lt;avr/wdt.h&gt;

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
  Serial.write(&quot;PickMasters booting\n&quot;);

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
    Serial.write(&quot;WARN: IMU not found — wrist auto-level disabled\n&quot;);
  }

  lastSerialRx = millis();
  pidLastTime  = millis();

  // Hardware watchdog: resets the MCU if loop() hangs for &gt;2 seconds
  wdt_enable(WDTO_2S);

  Serial.write(&quot;PickMasters ready\n&quot;);
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  wdt_reset();

  handleSerial();
  updateSafetyInterlock();

  if (imuOk &amp;&amp; millis() - lastIMU &gt;= 20) {   // 50 Hz IMU / wrist update
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
  if (dt &lt;= 0) dt = 0.02;
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
  if (millis() - lastSafetyCheck &lt; SAFETY_CHECK_MS) return;
  lastSafetyCheck = millis();

  bool hwClear     = (digitalRead(SAFETY_PIN) == HIGH);
  bool serialAlive = (millis() - lastSerialRx &lt; SERIAL_TIMEOUT_MS);
  bool sensorOk    = imuOk || !wristAutoMode;

  bool wasClear = safetyClear;
  safetyClear = hwClear &amp;&amp; serialAlive &amp;&amp; sensorOk;

  if (wasClear &amp;&amp; !safetyClear) {
    setMotorLeft(0);
    setMotorRight(0);
    digitalWrite(PUMP_PIN, HIGH);
    pumpOn = false;
    Serial.write(&quot;SAFETY: interlock OPEN — motors and pump disabled\n&quot;);
  }
  if (!wasClear &amp;&amp; safetyClear) {
    Serial.write(&quot;SAFETY: interlock CLEAR — movement enabled\n&quot;);
  }
}

// ── Serial parsing ────────────────────────────────────────────────────────────
void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == &#x27;\n&#x27;) {
      serialBuf[serialLen] = &#x27;\0&#x27;;
      parseCommand(serialBuf);
      serialLen = 0;
    } else if (c != &#x27;\r&#x27; &amp;&amp; serialLen &lt; sizeof(serialBuf) - 1) {
      serialBuf[serialLen++] = c;
    }
  }
}

// ── Checksum verification ─────────────────────────────────────────────────────
// If the command contains &#x27;*XX&#x27; suffix, verify XOR checksum; otherwise accept as-is.
bool verifyChecksum(char *cmd) {
  char *star = strchr(cmd, &#x27;*&#x27;);
  if (!star) return true;   // no checksum present — backward compatible

  byte computed = 0;
  for (char *p = cmd; p &lt; star; p++) computed ^= (byte)*p;

  unsigned int received;
  if (sscanf(star + 1, &quot;%02X&quot;, &amp;received) != 1) return false;
  *star = &#x27;\0&#x27;;   // strip checksum from command for parsing

  return (computed == (byte)received);
}

void parseCommand(char *cmd) {
  if (cmd[0] == &#x27;\0&#x27;) return;

  lastSerialRx = millis();

  if (!verifyChecksum(cmd)) {
    Serial.write(&quot;ERR: checksum mismatch\n&quot;);
    return;
  }

  if (cmd[0] == &#x27;S&#x27;) {
    int sh, el, wr, ha, l, r, p, w;
    if (sscanf(cmd, &quot;S%d,%d,%d,%d,M%d,%d,P%d,W%d&quot;,
               &amp;sh, &amp;el, &amp;wr, &amp;ha, &amp;l, &amp;r, &amp;p, &amp;w) == 8) {
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

  } else if (strcmp(cmd, &quot;cal&quot;) == 0) {
    calibrateIMU();
    pidIntegral  = 0.0;
    pidLastError = 0.0;
    Serial.write(&quot;cal ok\n&quot;);

  } else if (strcmp(cmd, &quot;status&quot;) == 0) {
    printStatus();

  } else if (strcmp(cmd, &quot;pid&quot;) == 0) {
    Serial.print(&quot;PID Kp=&quot;); Serial.print(pidKp, 2);
    Serial.print(&quot; Ki=&quot;);    Serial.print(pidKi, 2);
    Serial.print(&quot; Kd=&quot;);    Serial.print(pidKd, 2);
    Serial.write(&#x27;\n&#x27;);
  }
}

// ── DC motor control (L298N) ──────────────────────────────────────────────────
// Left wheel: ENA + IN1/IN2.  dir &gt; 0 = forward, dir &lt; 0 = backward, 0 = stop.
void setMotorLeft(int dir) {
  if (dir &gt; 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, DRIVE_SPEED);
  } else if (dir &lt; 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, DRIVE_SPEED);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

// Right wheel: ENB + IN3/IN4.  dir &gt; 0 = forward, dir &lt; 0 = backward, 0 = stop.
void setMotorRight(int dir) {
  if (dir &gt; 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, DRIVE_SPEED);
  } else if (dir &lt; 0) {
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

  // If the IMU doesn&#x27;t return all 6 bytes (unplugged/glitch), keep the last
  // pitch instead of computing garbage from -1 reads.
  if (Wire.requestFrom(MPU_ADDR, 6, true) != 6) return currentPitch;

  float accelX = (int16_t)(Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  float accelY = (int16_t)(Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  float accelZ = (int16_t)(Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  (void)accelY;

  return atan2(accelX, accelZ) * 180.0 / PI;
}

void calibrateIMU() {
  float sum = 0;
  for (int i = 0; i &lt; 50; i++) {
    sum += readPitch();
    delay(10);
  }
  pitchOffset = sum / 50.0;
}

// ── Status report (single compact line) ──────────────────────────────────────
void printStatus() {
  Serial.print(&quot;S:&quot;);    Serial.print(shoulderAngle);
  Serial.print(&quot; E:&quot;);   Serial.print(elbowAngle);
  Serial.print(&quot; W:&quot;);   Serial.print(wristAngle);
  Serial.print(&quot; H:&quot;);   Serial.print(handAngle);
  Serial.print(&quot; M:&quot;);   Serial.print(leftDir);
  Serial.print(&quot;,&quot;);     Serial.print(rightDir);
  Serial.print(&quot; P:&quot;);   Serial.print(pumpOn ? 1 : 0);
  Serial.print(&quot; MODE:&quot;); Serial.print(wristAutoMode ? &quot;AUTO/PID&quot; : &quot;MAN&quot;);
  Serial.print(&quot; IMU:&quot;);  Serial.print(imuOk ? &quot;OK&quot; : &quot;NONE&quot;);
  Serial.print(&quot; PITCH:&quot;); Serial.print(currentPitch - pitchOffset, 1);
  Serial.print(&quot; SAFE:&quot;);  Serial.print(safetyClear ? &quot;CLEAR&quot; : &quot;OPEN&quot;);
  Serial.print(&quot; WDT:ON&quot;);
  Serial.write(&#x27;\n&#x27;);
}
</code></pre>
</div>
</details>

## Arm Serial Protocol with Parity Checking

USB serial, 9600 baud. One line-oriented message sent only when a field changes (~50 ms cadence):

```
S<shoulder>,<elbow>,<wrist>,<hand>,M<left>,<right>,P<0|1>,W<0|1>*<XX>
```

Angles 0–180; motor directions in {−1, 0, +1}; P = pump on/off; W = wrist auto (1) or manual (0). The trailing `*XX` is an optional XOR checksum of all bytes before the asterisk. Mismatched checksums are rejected and an error returned. Commands without `*XX` are accepted for terminal diagnostics.

## Safety Interlock — Sequential Logic State Machine

A single boolean `safetyClear` gates the actuators — if low, pump and motors are forced safe regardless of last received command. Servos remain controllable (mechanically self-limited). `safetyClear` is recomputed every 50 ms as the AND of three conditions:

1. Hardware interlock pin A0 is HIGH (external pull-down to a normally-open e-stop).
2. Serial link is alive — no command received in > 2 s → assume host disconnected.
3. Sensors healthy — IMU must respond on I²C at boot, or auto-levelling is disabled.

Rising edge emits a confirmation over serial; falling edge emits a fault and immediately disables actuators.

## Hardware Watchdog Timer

AVR watchdog enabled at 2 s in `setup()` (`wdt_enable(WDTO_2S)`), reset every loop pass (`wdt_reset()`). A hung block (e.g. I²C wiring fault) triggers a reset back to the safe state. I²C library is also given a 3 ms bus timeout for graceful recovery from transient bus glitches.

## Wrist Auto-Levelling — PID Loop

PID compensator [1, Ch. 21]. Setpoint = 0° (level). Process variable = pitch from atan2 of accelerometer X and Z. Output drives the wrist servo around 90°. Gains tuned empirically: **Kp = 1.0, Ki = 0.05, Kd = 0.15.** Two safeguards: integral term clamped to ±30°·s (anti-windup); total output saturated to 0–180° servo range. Loop runs at 50 Hz — well above the MG996R's ~10 Hz internal bandwidth, satisfying Nyquist.

## Pick-and-Move Controller — Fuzzy Logic Vision-Guided Motion

PickAndMove.ino uses a Mamdani fuzzy controller. Input: absolute pixel error between target object centre and target-box centre. Output: motor PWM speed. Fuzzy is used because the pixel-error → speed relationship is non-linear (aggressive when far, gentle when close) — no single proportional gain works for both regimes.

- **Input membership functions (trapezoidal):** Small (0, 20, 40, 60); Medium (40, 60, 80, 120, 160); Large (120, 160, 200, ∞, ∞).
- **Output singletons (PWM 0–255):** Slow = 80, Medium = 150, Fast = 230.
- **Rules:** IF error Small → speed Slow; IF Medium → Medium; IF Large → Fast.
- **Defuzzification:** centre-of-gravity over the singletons weighted by firing strength.
- **Dead-zone fallback:** below a small firing-strength threshold, return a 60-PWM creep speed.

Motion is pulse-based: motors run 70 ms then auto-stop; the host re-sends pulses as long as motion is needed. A dropped serial link, host crash, or any communication failure halts the robot — a dead-man's-switch [1, Sec. 5.4]. YOLOv11 supplies the perception side [1, Ch. 22].

<details>
<summary>PickAndMove.ino — Arduino C | 203 lines</summary>
<div class="code-scroll">
<pre><code>// PickMasters — PickAndMove
// =========================
// Receives motion commands from detect_and_move.py over serial and drives the
// robot&#x27;s TWO DC motors (via an L298N) so the camera centres an item inside the
// target box.
//
//   * Spinning the base   -&gt; PIVOT the wheels (left motor and right motor turn
//                            in opposite directions) to line the item up on the
//                            VERTICAL line (X axis).
//   * Moving the base      -&gt; DRIVE both wheels the same direction to line the
//                            item up on the HORIZONTAL line (Y axis).
//
// Only the DC motors move here — the servos are left alone.
//
// Serial protocol (one command per line, 9600 baud):
//   PIVOT_LEFT  &lt;speed&gt;    spin base left   (speed = PWM 0-255)
//   PIVOT_RIGHT &lt;speed&gt;    spin base right
//   DRIVE_FWD   &lt;speed&gt;    move base forward
//   DRIVE_BACK  &lt;speed&gt;    move base backward
//   FUZZY_PIVOT &lt;error&gt;    fuzzy-logic pivot: error = signed pixel offset from centre
//   FUZZY_DRIVE &lt;error&gt;    fuzzy-logic drive: error = signed pixel offset from centre
//   STOP                   stop both motors
//
// Motion is PULSE based: each command runs the motors for PULSE_MS then stops
// automatically.  That way a dropped serial link can never leave the robot
// running away — the Python loop keeps sending pulses while it needs to move.
//
// Fuzzy Logic controller: maps pixel error to motor speed using trapezoidal
// membership functions for {Small, Medium, Large} error and {Slow, Medium, Fast}
// speed output, then defuzzifies via centre-of-gravity.

#include &lt;avr/wdt.h&gt;

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

  Serial.println(F(&quot;=== PickAndMove ready ===&quot;));
  Serial.println(F(&quot;Commands: PIVOT_LEFT/PIVOT_RIGHT/DRIVE_FWD/DRIVE_BACK &lt;speed&gt;,&quot;));
  Serial.println(F(&quot;          FUZZY_PIVOT/FUZZY_DRIVE &lt;error&gt;, STOP&quot;));
}

// ── Low level motor helpers ─────────────────────────────────────────────────
void setMotorLeft(int dir, int speed) {
  if (dir &gt; 0)      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, speed); }
  else if (dir &lt; 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, speed); }
  else              { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  analogWrite(ENA, 0); }
}

void setMotorRight(int dir, int speed) {
  if (dir &gt; 0)      { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, speed); }
  else if (dir &lt; 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, speed); }
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
  if (x &lt;= a || x &gt;= c) return 0.0;
  if (x &lt;= b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}

float fuzzyTrapezoid(float x, float a, float b, float c, float d) {
  if (x &lt;= a || x &gt;= d) return 0.0;
  if (x &gt;= b &amp;&amp; x &lt;= c) return 1.0;
  if (x &lt; b) return (x - a) / (b - a);
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

  if (denominator &lt; 0.001) return 60;   // dead zone fallback — minimal creep
  return constrain((int)(numerator / denominator), 0, 255);
}

// ── Serial parsing ───────────────────────────────────────────────────────────
int parseSpeed(const String &amp;input, int prefixLen) {
  int speed = input.substring(prefixLen).toInt();
  return constrain(speed, 0, 255);
}

int parseError(const String &amp;input, int prefixLen) {
  return input.substring(prefixLen).toInt();
}

void handleSerial() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil(&#x27;\n&#x27;);
  input.trim();
  if (input.length() == 0) return;

  if (input.equalsIgnoreCase(&quot;STOP&quot;)) {
    stopMotors();
    Serial.println(F(&quot;STOP&quot;));
  }
  else if (input.startsWith(&quot;FUZZY_PIVOT&quot;)) {
    int err = parseError(input, 11);
    int spd = fuzzyComputeSpeed(err);
    if (err &lt; 0) pivotLeft(spd); else pivotRight(spd);
    Serial.print(F(&quot;FUZZY_PIVOT err=&quot;)); Serial.print(err);
    Serial.print(F(&quot; spd=&quot;));            Serial.println(spd);
  }
  else if (input.startsWith(&quot;FUZZY_DRIVE&quot;)) {
    int err = parseError(input, 11);
    int spd = fuzzyComputeSpeed(err);
    if (err &gt; 0) driveForward(spd); else driveBackward(spd);
    Serial.print(F(&quot;FUZZY_DRIVE err=&quot;)); Serial.print(err);
    Serial.print(F(&quot; spd=&quot;));            Serial.println(spd);
  }
  else if (input.startsWith(&quot;PIVOT_LEFT&quot;)) {
    int s = parseSpeed(input, 10);
    pivotLeft(s);
    Serial.print(F(&quot;PIVOT_LEFT &quot;));  Serial.println(s);
  }
  else if (input.startsWith(&quot;PIVOT_RIGHT&quot;)) {
    int s = parseSpeed(input, 11);
    pivotRight(s);
    Serial.print(F(&quot;PIVOT_RIGHT &quot;)); Serial.println(s);
  }
  else if (input.startsWith(&quot;DRIVE_FWD&quot;)) {
    int s = parseSpeed(input, 9);
    driveForward(s);
    Serial.print(F(&quot;DRIVE_FWD &quot;));   Serial.println(s);
  }
  else if (input.startsWith(&quot;DRIVE_BACK&quot;)) {
    int s = parseSpeed(input, 10);
    driveBackward(s);
    Serial.print(F(&quot;DRIVE_BACK &quot;));  Serial.println(s);
  }
  else {
    Serial.print(F(&quot;Unknown command: &quot;));
    Serial.println(input);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  wdt_reset();
  handleSerial();

  if (stopAt != 0 &amp;&amp; millis() &gt;= stopAt) {
    stopMotors();
  }
}
</code></pre>
</div>
</details>

## Choice of Programming Language — Assembly vs C

C used for both sketches. Assembly was considered for I²C IMU reads and the inner fuzzy defuzzification loop. Rejected both: I²C reads complete in well under the 20 ms loop budget; software-float defuzzification takes under 1 ms. No measurable timing benefit, and the legibility cost of mixed-language code is high. Standard guidance from [1, Ch. 12] for modern 8-bit MCUs.

## Host Computer Software

Two host programs, one per mode. Manual: Processing sketch (Appendix C.3) using GameControlPlus; reads the Bluetooth gamepad, applies the XOR checksum, sends a frame at most every 50 ms only if a field changed. Automatic: `detect_and_move.py` (Appendix C.4) using OpenCV + YOLOv11; locks onto the highest-confidence detection above 50%, computes signed pixel errors, and sends FUZZY_PIVOT / FUZZY_DRIVE / STOP commands.

## Distributed Control Architecture

Current architecture is a degenerate distributed system [1, Sec. 15.2]: host runs high-level logic, Arduino runs low-level real-time control, single USB serial link between them. Adequate for the benchtop demo, not assembly-line ready. Three extensions considered for industrial use: CAN bus for noise immunity and direct conveyor coordination; Modbus RTU layered over the existing USB serial; ROS 2 topic over Ethernet (the ROS 2 twin in §10.4 is already subscriber-ready). None are in the prototype.

## Industrial Transition Note

A real deployment would replace the Arduino with a PLC [1, Ch. 14] for hardened I/O, noise immunity, and reliability. The watchdog and safety interlock would map to standard PLC equivalents; the XOR checksum would fold into the fieldbus error detection. The Arduino is fine for the prototype but would not survive production.

---

[← Back to Home](../index.md)
