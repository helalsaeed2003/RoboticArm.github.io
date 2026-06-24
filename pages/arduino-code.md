# Development Sketches

[← Back to Home](../index.md)

---

These are the development-stage Arduino and Processing sketches created during the build process. They test individual components, iterate on control approaches, and document the evolution from basic servo tests to the final integrated firmware. The production code is in [Final Code](final-code.md).

---

## [`ArmController.ino`](../Arduino%20Code/ArmController/ArmController.ino)

An earlier version of the manual-mode firmware before PID wrist levelling and the safety interlock were added. Drives servos and DC motors via serial commands from the Processing gamepad sketch. Used during mid-project integration testing.

<details>
<summary>Arduino C | 267 lines</summary>
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
//   L298N driver: ENA=5, IN1=2, IN2=4 (left wheel);
//                 ENB=6, IN3=7, IN4=8 (right wheel).
//
// Serial protocol (9600 baud, newline-terminated, ONE combined message per frame):
//   S&lt;shoulder&gt;,&lt;elbow&gt;,&lt;wrist&gt;,&lt;hand&gt;,M&lt;left&gt;,&lt;right&gt;,P&lt;0|1&gt;,W&lt;0|1&gt;
//     servo angles 0..180, motor directions -1/0/1, P1 = pump on,
//     W1 = wrist AUTO (IMU leveling), W0 = wrist MANUAL
//   cal     — re-zero IMU pitch offset (sent by the DriveControl CalButton)
//   status  — print all current angles and states (single compact line)

#include &lt;Wire.h&gt;
#include &lt;Servo.h&gt;

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
  Serial.write(&quot;PickMasters booting\n&quot;);   // instant proof serial is alive

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
  // setWireTimeout(us, reset_on_timeout): bail out and reset the bus instead
  // of blocking forever when the MPU6050 doesn&#x27;t respond.
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
    // No IMU: keep running everything else, just don&#x27;t auto-level the wrist.
    wristAutoMode = false;
    Serial.write(&quot;WARN: IMU not found — wrist auto-level disabled\n&quot;);
  }

  Serial.write(&quot;PickMasters ready\n&quot;);
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  handleSerial();

  if (imuOk &amp;&amp; millis() - lastIMU &gt;= 20) {   // 50 Hz IMU / wrist update
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
    if (c == &#x27;\n&#x27;) {
      serialBuf[serialLen] = &#x27;\0&#x27;;
      parseCommand(serialBuf);
      serialLen = 0;
    } else if (c != &#x27;\r&#x27; &amp;&amp; serialLen &lt; sizeof(serialBuf) - 1) {
      serialBuf[serialLen++] = c;
    }
  }
}

void parseCommand(char *cmd) {
  if (cmd[0] == &#x27;\0&#x27;) return;

  if (cmd[0] == &#x27;S&#x27;) {
    // Combined frame: S&lt;sh&gt;,&lt;el&gt;,&lt;wr&gt;,&lt;ha&gt;,M&lt;l&gt;,&lt;r&gt;,P&lt;p&gt;,W&lt;w&gt;
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

  } else if (strcmp(cmd, &quot;cal&quot;) == 0) {
    calibrateIMU();
    Serial.write(&quot;cal ok\n&quot;);

  } else if (strcmp(cmd, &quot;status&quot;) == 0) {
    printStatus();
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
  Serial.print(&quot; MODE:&quot;); Serial.print(wristAutoMode ? &quot;AUTO&quot; : &quot;MAN&quot;);
  Serial.print(&quot; IMU:&quot;);  Serial.print(imuOk ? &quot;OK&quot; : &quot;NONE&quot;);
  Serial.print(&quot; PITCH:&quot;); Serial.print(currentPitch - pitchOffset, 1);
  Serial.write(&#x27;\n&#x27;);
}</code></pre>
</div>
</details>

---

## [`ComponentTest.ino`](../Arduino%20Code/ComponentTest/ComponentTest.ino)

Interactive bench-test sketch for every hardware component. Open the Serial Monitor, type a command, and individually test each servo, DC motor direction, pump relay, and IMU reading. Essential for verifying wiring before running the full firmware.

<details>
<summary>Arduino C | 251 lines</summary>
<div class="code-scroll">
<pre><code>// PickMasters — ComponentTest
// Interactive bench test for every component, using the CURRENT hardware setup
// (standalone L298N motor driver + servos on digital pins).
//
// Open the Serial Monitor at 9600 baud, set line ending to &quot;Newline&quot; (or just
// send single characters), and press a key to run each test:
//
//   1  — Shoulder servo sweep  (pin 9)
//   2  — Elbow servo sweep     (pin 10)
//   3  — Wrist servo sweep     (pin 11)
//   4  — Hand servo sweep      (pin 12)
//   5  — All servos sweep together
//   p  — Pump relay ON/OFF toggle      (pin 3, ACTIVE LOW)
//   l  — Left motor: FWD -&gt; REV -&gt; STOP (ENA=5, IN1=2, IN2=4 -&gt; OUT1/OUT2)
//   r  — Right motor: FWD -&gt; REV -&gt; STOP (ENB=6, IN3=7, IN4=8 -&gt; OUT3/OUT4)
//   m  — Both motors: FWD -&gt; REV -&gt; PIVOT L -&gt; PIVOT R -&gt; STOP
//   i  — Stream IMU pitch for 5 s         (MPU6050 on A4/A5)
//   b  — Watch calibration button for 5 s (pin A0, INPUT_PULLUP)
//   h  — Print this menu again

#include &lt;Wire.h&gt;
#include &lt;Servo.h&gt;

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

  Serial.println(F(&quot;PickMasters component test ready&quot;));
  printMenu();
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  if (!Serial.available()) return;
  char c = (char)Serial.read();

  switch (c) {
    case &#x27;1&#x27;: sweepServo(shoulderServo, &quot;Shoulder&quot;); break;
    case &#x27;2&#x27;: sweepServo(elbowServo,    &quot;Elbow&quot;);    break;
    case &#x27;3&#x27;: sweepServo(wristServo,    &quot;Wrist&quot;);    break;
    case &#x27;4&#x27;: sweepServo(handServo,     &quot;Hand&quot;);     break;
    case &#x27;5&#x27;: sweepAllServos();                      break;
    case &#x27;p&#x27;: togglePump();                          break;
    case &#x27;l&#x27;: testMotorLeft();                       break;
    case &#x27;r&#x27;: testMotorRight();                      break;
    case &#x27;m&#x27;: testBothMotors();                      break;
    case &#x27;i&#x27;: streamIMU();                           break;
    case &#x27;b&#x27;: watchButton();                         break;
    case &#x27;h&#x27;: printMenu();                           break;
    case &#x27;\n&#x27;: case &#x27;\r&#x27;: break;   // ignore line endings
    default:
      Serial.print(F(&quot;Unknown command: &quot;));
      Serial.println(c);
      break;
  }
}

// ── Servo tests ───────────────────────────────────────────────────────────────
void sweepServo(Servo &amp;s, const char *name) {
  Serial.print(F(&quot;Sweeping &quot;));
  Serial.print(name);
  Serial.println(F(&quot; 0 -&gt; 180 -&gt; 90&quot;));
  for (int a = 0; a &lt;= 180; a += 2) { s.write(a); delay(15); }
  for (int a = 180; a &gt;= 0; a -= 2) { s.write(a); delay(15); }
  s.write(90);
  Serial.println(F(&quot;  done (returned to 90)&quot;));
}

void sweepAllServos() {
  Serial.println(F(&quot;Sweeping ALL servos 0 -&gt; 180 -&gt; 90&quot;));
  for (int a = 0; a &lt;= 180; a += 2) {
    shoulderServo.write(a); elbowServo.write(a);
    wristServo.write(a);    handServo.write(a);
    delay(15);
  }
  for (int a = 180; a &gt;= 0; a -= 2) {
    shoulderServo.write(a); elbowServo.write(a);
    wristServo.write(a);    handServo.write(a);
    delay(15);
  }
  shoulderServo.write(90); elbowServo.write(90);
  wristServo.write(90);    handServo.write(90);
  Serial.println(F(&quot;  done (all returned to 90)&quot;));
}

// ── Pump test ─────────────────────────────────────────────────────────────────
void togglePump() {
  pumpOn = !pumpOn;
  digitalWrite(PUMP_PIN, pumpOn ? LOW : HIGH);   // active LOW
  Serial.print(F(&quot;Pump &quot;));
  Serial.println(pumpOn ? F(&quot;ON&quot;) : F(&quot;OFF&quot;));
}

// ── Motor tests ───────────────────────────────────────────────────────────────
void setMotorLeft(int dir) {
  if (dir &gt; 0)      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, DRIVE_SPEED); }
  else if (dir &lt; 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, DRIVE_SPEED); }
  else              { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  analogWrite(ENA, 0); }
}

void setMotorRight(int dir) {
  if (dir &gt; 0)      { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, DRIVE_SPEED); }
  else if (dir &lt; 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, DRIVE_SPEED); }
  else              { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  analogWrite(ENB, 0); }
}

void stopMotors() { setMotorLeft(0); setMotorRight(0); }

void testMotorLeft() {
  Serial.println(F(&quot;LEFT motor (OUT1/OUT2): FWD 1.5s&quot;));
  setMotorLeft(1);  delay(1500);
  Serial.println(F(&quot;  REV 1.5s&quot;));
  setMotorLeft(-1); delay(1500);
  setMotorLeft(0);
  Serial.println(F(&quot;  STOP&quot;));
}

void testMotorRight() {
  Serial.println(F(&quot;RIGHT motor (OUT3/OUT4): FWD 1.5s&quot;));
  setMotorRight(1);  delay(1500);
  Serial.println(F(&quot;  REV 1.5s&quot;));
  setMotorRight(-1); delay(1500);
  setMotorRight(0);
  Serial.println(F(&quot;  STOP&quot;));
}

void testBothMotors() {
  Serial.println(F(&quot;BOTH: forward 1.5s&quot;));
  setMotorLeft(1);  setMotorRight(1);  delay(1500);
  Serial.println(F(&quot;  reverse 1.5s&quot;));
  setMotorLeft(-1); setMotorRight(-1); delay(1500);
  Serial.println(F(&quot;  pivot LEFT 1.5s&quot;));
  setMotorLeft(-1); setMotorRight(1);  delay(1500);
  Serial.println(F(&quot;  pivot RIGHT 1.5s&quot;));
  setMotorLeft(1);  setMotorRight(-1); delay(1500);
  stopMotors();
  Serial.println(F(&quot;  STOP&quot;));
}

// ── IMU test ──────────────────────────────────────────────────────────────────
float readPitch() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  float accelX = (int16_t)(Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  float accelY = (int16_t)(Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  float accelZ = (int16_t)(Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  (void)accelY;
  return atan2(accelX, accelZ) * 180.0 / PI;
}

void streamIMU() {
  Serial.println(F(&quot;Streaming IMU pitch for 5 s — tilt the arm...&quot;));
  unsigned long t0 = millis();
  while (millis() - t0 &lt; 5000) {
    Serial.print(F(&quot;  pitch = &quot;));
    Serial.print(readPitch(), 1);
    Serial.println(F(&quot; deg&quot;));
    delay(200);
  }
  Serial.println(F(&quot;  done&quot;));
}

// ── Calibration button test ──────────────────────────────────────────────────
void watchButton() {
  Serial.println(F(&quot;Watching cal button (pin A0) for 5 s — press it...&quot;));
  bool prev = HIGH;
  unsigned long t0 = millis();
  while (millis() - t0 &lt; 5000) {
    bool btn = digitalRead(CAL_BUTTON);
    if (btn == LOW &amp;&amp; prev == HIGH) Serial.println(F(&quot;  BUTTON PRESSED&quot;));
    prev = btn;
    delay(10);
  }
  Serial.println(F(&quot;  done&quot;));
}

// ── Menu ──────────────────────────────────────────────────────────────────────
void printMenu() {
  Serial.println(F(&quot;------ component test menu ------&quot;));
  Serial.println(F(&quot; 1  Shoulder servo (pin 9)&quot;));
  Serial.println(F(&quot; 2  Elbow servo    (pin 10)&quot;));
  Serial.println(F(&quot; 3  Wrist servo    (pin 11)&quot;));
  Serial.println(F(&quot; 4  Hand servo     (pin 12)&quot;));
  Serial.println(F(&quot; 5  All servos together&quot;));
  Serial.println(F(&quot; p  Pump relay toggle (pin 3)&quot;));
  Serial.println(F(&quot; l  Left motor  (OUT1/OUT2)&quot;));
  Serial.println(F(&quot; r  Right motor (OUT3/OUT4)&quot;));
  Serial.println(F(&quot; m  Both motors sequence&quot;));
  Serial.println(F(&quot; i  IMU pitch stream (5 s)&quot;));
  Serial.println(F(&quot; b  Cal button watch (pin A0, 5 s)&quot;));
  Serial.println(F(&quot; h  Show this menu&quot;));
  Serial.println(F(&quot;--------------------------------&quot;));
}</code></pre>
</div>
</details>

---

## [`AutoMode.ino`](../Arduino%20Code/AutoMode/AutoMode.ino)

Early automatic-mode prototype. Combines servo control with basic serial commands from the vision script. Predates the fuzzy-logic controller — uses fixed-speed motor commands instead.

<details>
<summary>Arduino C | 185 lines</summary>
<div class="code-scroll">
<pre><code>#include &lt;Wire.h&gt;
#include &lt;Servo.h&gt;

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

  Serial.println(&quot;=== Arm Controller Ready ===&quot;);
  Serial.println(&quot;Format: &lt;servo&gt; &lt;angle&gt;&quot;);
  Serial.println(&quot;  1=Base  2=Shoulder  3=Elbow&quot;);
  Serial.println(&quot;  Wrist is auto-leveled&quot;);
  Serial.println(&quot;Type &#x27;cal&#x27; to re-zero IMU&quot;);
  Serial.println(&quot;Type &#x27;status&#x27; for current angles&quot;);
  Serial.println(&quot;============================\n&quot;);
}

// ---------- MPU6050 ----------

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  accelX = (Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  accelY = (Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  accelZ = (Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
}

float getPitch() {
  return atan2(accelX, accelZ) * 180.0 / PI;
}

void calibrate() {
  float sum = 0;
  for (int i = 0; i &lt; 50; i++) {
    readMPU();
    sum += getPitch();
    delay(10);
  }
  pitchOffset = sum / 50.0;
  Serial.print(&quot;Calibrated. Offset: &quot;);
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

  String input = Serial.readStringUntil(&#x27;\n&#x27;);
  input.trim();

  if (input.equalsIgnoreCase(&quot;cal&quot;)) {
    Serial.println(&quot;Re-calibrating...&quot;);
    calibrate();
    return;
  }

  if (input.equalsIgnoreCase(&quot;status&quot;)) {
    printStatus();
    return;
  }

  int spaceIndex = input.indexOf(&#x27; &#x27;);
  if (spaceIndex == -1) {
    Serial.println(&quot;Invalid format. Use: &lt;servo&gt; &lt;angle&gt;&quot;);
    return;
  }

  int servoNum = input.substring(0, spaceIndex).toInt();
  int angle    = input.substring(spaceIndex + 1).toInt();

  if (servoNum &lt; 1 || servoNum &gt; 3) {
    Serial.println(&quot;Servo must be 1-3. Wrist is automatic.&quot;);
    return;
  }
  if (angle &lt; 0 || angle &gt; 180) {
    Serial.println(&quot;Angle must be 0-180.&quot;);
    return;
  }

  switch (servoNum) {
    case 1:
      baseAngle = angle;
      baseServo.write(angle);
      Serial.print(&quot;Base -&gt; &quot;);
      break;
    case 2:
      shoulderAngle = angle;
      shoulderServo.write(angle);
      Serial.print(&quot;Shoulder -&gt; &quot;);
      break;
    case 3:
      elbowAngle = angle;
      elbowServo.write(angle);
      Serial.print(&quot;Elbow -&gt; &quot;);
      break;
  }
  Serial.print(angle);
  Serial.println(&quot; deg&quot;);
}

void printStatus() {
  Serial.println(&quot;----- Current Angles -----&quot;);
  Serial.print(&quot;  1) Base     : &quot;); Serial.println(baseAngle);
  Serial.print(&quot;  2) Shoulder : &quot;); Serial.println(shoulderAngle);
  Serial.print(&quot;  3) Elbow    : &quot;); Serial.println(elbowAngle);
  Serial.print(&quot;  4) Wrist    : &quot;); Serial.print(wristCenter);
  Serial.print(&quot; + pitch correction: &quot;); Serial.println(currentPitch);
  Serial.println(&quot;--------------------------&quot;);
}

// ---------- Main Loop ----------

void loop() {
  if (digitalRead(CALIBRATE_BTN) == LOW) {
    Serial.println(&quot;Re-calibrating...&quot;);
    calibrate();
    delay(500);
  }

  handleSerial();
  updateWrist();

  delay(20);
}</code></pre>
</div>
</details>

---

## [`CameraVisionTest.ino`](../Arduino%20Code/CameraVisionTest/CameraVisionTest.ino)

Test firmware for validating the camera-to-Arduino communication loop. Accepts vision commands over serial and drives servos and motors in response. Used to debug the serial protocol between <code>detect_and_move.py</code> and the Arduino.

<details>
<summary>Arduino C | 204 lines</summary>
<div class="code-scroll">
<pre><code>#include &lt;Wire.h&gt;
#include &lt;Servo.h&gt;

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

  Serial.println(&quot;=== Arm Controller Ready ===&quot;);
  Serial.println(&quot;Format: &lt;servo&gt; &lt;angle&gt;&quot;);
  Serial.println(&quot;  1=Base  2=Shoulder  3=Elbow&quot;);
  Serial.println(&quot;  Wrist is auto-leveled&quot;);
  Serial.println(&quot;Type &#x27;cal&#x27; to re-zero IMU&quot;);
  Serial.println(&quot;Type &#x27;status&#x27; for current angles&quot;);
  Serial.println(&quot;============================\n&quot;);
}

// ---------- MPU6050 ----------

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  accelX = (Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  accelY = (Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  accelZ = (Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
}

float getPitch() {
  return atan2(accelX, accelZ) * 180.0 / PI;
}

void calibrate() {
  float sum = 0;
  for (int i = 0; i &lt; 50; i++) {
    readMPU();
    sum += getPitch();
    delay(10);
  }
  pitchOffset = sum / 50.0;
  Serial.print(&quot;Calibrated. Offset: &quot;);
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

  String input = Serial.readStringUntil(&#x27;\n&#x27;);
  input.trim();

  if (input.equalsIgnoreCase(&quot;cal&quot;)) {
    Serial.println(&quot;Re-calibrating...&quot;);
    calibrate();
    return;
  }

  if (input.equalsIgnoreCase(&quot;status&quot;)) {
    printStatus();
    return;
  }

  // BASE LEFT and BASE RIGHT from Python
  if (input.startsWith(&quot;BASE LEFT&quot;)) {
    int speed = input.substring(10).toInt();
    if (speed &lt;= 0) speed = 5;
    baseAngle = constrain(baseAngle - speed, 0, 180);
    baseServo.write(baseAngle);
    Serial.print(&quot;Base Left -&gt; &quot;);
    Serial.println(baseAngle);
    return;
  }

  if (input.startsWith(&quot;BASE RIGHT&quot;)) {
    int speed = input.substring(11).toInt();
    if (speed &lt;= 0) speed = 5;
    baseAngle = constrain(baseAngle + speed, 0, 180);
    baseServo.write(baseAngle);
    Serial.print(&quot;Base Right -&gt; &quot;);
    Serial.println(baseAngle);
    return;
  }

  // Manual servo control
  int spaceIndex = input.indexOf(&#x27; &#x27;);
  if (spaceIndex == -1) {
    Serial.println(&quot;Invalid format. Use: &lt;servo&gt; &lt;angle&gt;&quot;);
    return;
  }

  int servoNum = input.substring(0, spaceIndex).toInt();
  int angle    = input.substring(spaceIndex + 1).toInt();

  if (servoNum &lt; 1 || servoNum &gt; 3) {
    Serial.println(&quot;Servo must be 1-3. Wrist is automatic.&quot;);
    return;
  }
  if (angle &lt; 0 || angle &gt; 180) {
    Serial.println(&quot;Angle must be 0-180.&quot;);
    return;
  }

  switch (servoNum) {
    case 1:
      baseAngle = angle;
      baseServo.write(angle);
      Serial.print(&quot;Base -&gt; &quot;);
      break;
    case 2:
      shoulderAngle = angle;
      shoulderServo.write(angle);
      Serial.print(&quot;Shoulder -&gt; &quot;);
      break;
    case 3:
      elbowAngle = angle;
      elbowServo.write(angle);
      Serial.print(&quot;Elbow -&gt; &quot;);
      break;
  }
  Serial.print(angle);
  Serial.println(&quot; deg&quot;);
}

void printStatus() {
  Serial.println(&quot;----- Current Angles -----&quot;);
  Serial.print(&quot;  1) Base     : &quot;); Serial.println(baseAngle);
  Serial.print(&quot;  2) Shoulder : &quot;); Serial.println(shoulderAngle);
  Serial.print(&quot;  3) Elbow    : &quot;); Serial.println(elbowAngle);
  Serial.print(&quot;  4) Wrist    : &quot;); Serial.print(wristCenter);
  Serial.print(&quot; + pitch correction: &quot;); Serial.println(currentPitch);
  Serial.println(&quot;--------------------------&quot;);
}

// ---------- Main Loop ----------

void loop() {
  if (digitalRead(CALIBRATE_BTN) == LOW) {
    Serial.println(&quot;Re-calibrating...&quot;);
    calibrate();
    delay(500);
  }

  handleSerial();
  updateWrist();

  delay(20);
}</code></pre>
</div>
</details>

---

## [`ManualDrive.ino`](../Arduino%20Code/ManualDrive/ManualDrive.ino)

The original manual-drive firmware written for the L293D motor shield (before it shorted). Uses the AFMotor library. Kept in the repo as a record of the original hardware design and the lesson learned about driver current ratings.

<details>
<summary>Arduino C | 224 lines</summary>
<div class="code-scroll">
<pre><code>// PickMasters — ManualDrive firmware
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
//   M &lt;left&gt; &lt;right&gt;   — DC motor speeds, -255..255  (differential drive)
//   2 &lt;angle&gt;          — Shoulder servo, 0..180 deg
//   3 &lt;angle&gt;          — Elbow servo,    0..180 deg
//   P 1 / P 0          — Pump relay ON / OFF
//   cal                — Re-zero IMU pitch
//   stop               — Stop motors
//   status             — Print all joint states

#include &lt;Wire.h&gt;
#include &lt;Servo.h&gt;
#include &lt;AFMotor.h&gt;   // Adafruit Motor Shield v1 library

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
String serialBuf = &quot;&quot;;

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
  Serial.println(&quot;PickMasters ready&quot;);
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  handleSerial();
  handleCalButton();

  if (millis() - lastIMU &gt;= 20) {   // 50 Hz wrist update
    lastIMU = millis();
    updateWrist();
  }
}

// ── Serial parsing ────────────────────────────────────────────────────────────
void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == &#x27;\n&#x27;) {
      parseCommand(serialBuf);
      serialBuf = &quot;&quot;;
    } else if (c != &#x27;\r&#x27;) {
      serialBuf += c;
    }
  }
}

void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.startsWith(&quot;M &quot;)) {
    int sep = cmd.indexOf(&#x27; &#x27;, 2);
    if (sep &lt; 0) return;
    int l = cmd.substring(2, sep).toInt();
    int r = cmd.substring(sep + 1).toInt();
    driveMotors(l, r);

  } else if (cmd.startsWith(&quot;P &quot;)) {
    bool on = (cmd.substring(2).toInt() != 0);
    digitalWrite(PUMP_PIN, on ? HIGH : LOW);
    Serial.println(on ? &quot;Pump ON&quot; : &quot;Pump OFF&quot;);

  } else if (cmd.startsWith(&quot;2 &quot;)) {
    shoulderAngle = constrain(cmd.substring(2).toInt(), 0, 180);
    shoulderServo.write(shoulderAngle);
    Serial.println(&quot;Shoulder -&gt; &quot; + String(shoulderAngle));

  } else if (cmd.startsWith(&quot;3 &quot;)) {
    elbowAngle = constrain(cmd.substring(2).toInt(), 0, 180);
    elbowServo.write(elbowAngle);
    Serial.println(&quot;Elbow -&gt; &quot; + String(elbowAngle));

  } else if (cmd == &quot;cal&quot;) {
    calibrateIMU();
    Serial.println(&quot;IMU calibrated&quot;);

  } else if (cmd == &quot;stop&quot;) {
    driveMotors(0, 0);
    Serial.println(&quot;Motors stopped&quot;);

  } else if (cmd == &quot;status&quot;) {
    Serial.println(&quot;----- Status -----&quot;);
    Serial.println(&quot;Shoulder : &quot; + String(shoulderAngle));
    Serial.println(&quot;Elbow    : &quot; + String(elbowAngle));
    Serial.println(&quot;Wrist    : &quot; + String(wristAngle) + &quot; (auto)&quot;);
    Serial.println(&quot;------------------&quot;);
  }
}

// ── DC motor control ──────────────────────────────────────────────────────────
void driveMotors(int l, int r) {
  setMotor(motorLeft,  l);
  setMotor(motorRight, r);
}

void setMotor(AF_DCMotor &amp;m, int spd) {
  if (spd &gt; 0) {
    m.setSpeed(min(spd, 255));
    m.run(FORWARD);
  } else if (spd &lt; 0) {
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

  int16_t ax = ((int16_t)Wire.read() &lt;&lt; 8) | Wire.read();
  int16_t ay = ((int16_t)Wire.read() &lt;&lt; 8) | Wire.read();
  int16_t az = ((int16_t)Wire.read() &lt;&lt; 8) | Wire.read();

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
  if (btn == LOW &amp;&amp; prevCalBtn == HIGH) {  // falling edge = pressed
    calibrateIMU();
    Serial.println(&quot;IMU calibrated (button)&quot;);
  }
  prevCalBtn = btn;
}</code></pre>
</div>
</details>

---

## [`SelfLevling.ino`](../Arduino%20Code/SelfLevling/SelfLevling.ino)

Standalone wrist auto-levelling test. Reads the MPU6050 over I²C, computes pitch from accelerometer data, and drives a single servo to keep the wrist level. Used to tune the PID gains before integrating into the full ArmController.

<details>
<summary>Arduino C | 101 lines</summary>
<div class="code-scroll">
<pre><code>#include &lt;Wire.h&gt;
#include &lt;Servo.h&gt;

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

  Serial.println(&quot;Wrist Leveling Active&quot;);
  Serial.println(&quot;Type &#x27;cal&#x27; to re-zero&quot;);
  Serial.println(&quot;======================&quot;);
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); // Only need accel data

  accelX = (Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  accelY = (Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  accelZ = (Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
}

float getPitch() {
  return atan2(accelX, accelZ) * 180.0 / PI;
}

void calibrate() {
  // Average multiple readings for a stable offset
  float sum = 0;
  for (int i = 0; i &lt; 50; i++) {
    readMPU();
    sum += getPitch();
    delay(10);
  }
  pitchOffset = sum / 50.0;
  Serial.print(&quot;Calibrated. Offset: &quot;);
  Serial.println(pitchOffset);
}

void loop() {
  // Check for calibration button
  if (digitalRead(CALIBRATE_BTN) == LOW) {
    Serial.println(&quot;Re-calibrating...&quot;);
    calibrate();
    delay(500); // Debounce
  }

  // Check for serial calibration command
  if (Serial.available()) {
    String input = Serial.readStringUntil(&#x27;\n&#x27;);
    input.trim();
    if (input.equalsIgnoreCase(&quot;cal&quot;)) {
      Serial.println(&quot;Re-calibrating...&quot;);
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

  Serial.print(&quot;Pitch: &quot;);
  Serial.print(currentPitch);
  Serial.print(&quot;  Servo: &quot;);
  Serial.println(servoAngle);

  delay(20); // 50Hz update rate
}</code></pre>
</div>
</details>

---

## [`PickAndMove.ino`](../Arduino%20Code/PickAndMove/PickAndMove.ino)

Earlier version of the automatic-mode firmware. Simpler command set without the fuzzy-logic speed controller — uses fixed PWM values for each direction command.

<details>
<summary>Arduino C | 131 lines</summary>
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

  Serial.println(F(&quot;=== PickAndMove ready ===&quot;));
  Serial.println(F(&quot;Commands: PIVOT_LEFT/PIVOT_RIGHT/DRIVE_FWD/DRIVE_BACK &lt;speed&gt;, STOP&quot;));
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

// ── Serial parsing ───────────────────────────────────────────────────────────
int parseSpeed(const String &amp;input, int prefixLen) {
  int speed = input.substring(prefixLen).toInt();
  return constrain(speed, 0, 255);
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
  handleSerial();

  // Auto-stop when the current pulse expires.
  if (stopAt != 0 &amp;&amp; millis() &gt;= stopAt) {
    stopMotors();
  }
}</code></pre>
</div>
</details>

---

## [`ServoTest.ino`](../Arduino%20Code/ServoTest/ServoTest.ino)

Minimal sketch to sweep two servos through their range. Used to verify servo wiring, confirm PWM pin assignments, and check for mechanical binding in the 3D-printed joints.

<details>
<summary>Arduino C | 80 lines</summary>
<div class="code-scroll">
<pre><code>#include &lt;Servo.h&gt;

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

  Serial.println(&quot;=== 2-DOF Servo Controller ===&quot;);
  Serial.println(&quot;Format:  &lt;servo&gt; &lt;angle&gt;&quot;);
  Serial.println(&quot;  servo : 1=Base  2=Shoulder&quot;);
  Serial.println(&quot;  angle : 0 - 270&quot;);
  Serial.println(&quot;Example: 2 45   (moves Shoulder to 45 deg)&quot;);
  Serial.println(&quot;Type &#x27;status&#x27; to see current angles.&quot;);
  Serial.println(&quot;==============================\n&quot;);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil(&#x27;\n&#x27;);
    input.trim();

    if (input.equalsIgnoreCase(&quot;status&quot;)) {
      printStatus();
      return;
    }

    int spaceIndex = input.indexOf(&#x27; &#x27;);
    if (spaceIndex == -1) {
      Serial.println(&quot;Invalid format. Use: &lt;servo&gt; &lt;angle&gt;&quot;);
      return;
    }

    int servoNum = input.substring(0, spaceIndex).toInt();
    int angle    = input.substring(spaceIndex + 1).toInt();

    if (servoNum &lt; 1 || servoNum &gt; 2) {
      Serial.println(&quot;Servo must be 1 or 2.&quot;);
      return;
    }
    if (angle &lt; 0 || angle &gt; 270) {
      Serial.println(&quot;Angle must be 0-270.&quot;);
      return;
    }

    switch (servoNum) {
      case 1:
        baseAngle = angle;
        baseServo.write(angle);
        Serial.print(&quot;Base -&gt; &quot;);
        break;
      case 2:
        shoulderAngle = angle;
        shoulderServo.write(angle);
        Serial.print(&quot;Shoulder -&gt; &quot;);
        break;
    }
    Serial.print(angle);
    Serial.println(&quot; deg&quot;);
  }
}

void printStatus() {
  Serial.println(&quot;----- Current Angles -----&quot;);
  Serial.print(&quot;  1) Base     : &quot;); Serial.println(baseAngle);
  Serial.print(&quot;  2) Shoulder : &quot;); Serial.println(shoulderAngle);
  Serial.println(&quot;--------------------------&quot;);
}</code></pre>
</div>
</details>

---

## [`SesnsorTest.ino`](../Arduino%20Code/SesnsorTest/SesnsorTest.ino)

IMU sensor test. Reads raw accelerometer and gyroscope values from the MPU6050 and prints them to the Serial Monitor. Used to verify I²C communication and check sensor orientation before writing the pitch calculation.

<details>
<summary>Arduino C | 67 lines</summary>
<div class="code-scroll">
<pre><code>#include &lt;Wire.h&gt;

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

  // Verify it&#x27;s awake by reading WHO_AM_I register
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  byte whoAmI = Wire.read();

  Serial.print(&quot;WHO_AM_I: 0x&quot;);
  Serial.println(whoAmI, HEX);

  if (whoAmI == 0x68) {
    Serial.println(&quot;MPU6050 confirmed and ready!&quot;);
  } else {
    Serial.println(&quot;Unexpected response — check sensor.&quot;);
  }

  Serial.println(&quot;==============&quot;);
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  accelX = (Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  accelY = (Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  accelZ = (Wire.read() &lt;&lt; 8 | Wire.read()) / 16384.0;
  temp   = (Wire.read() &lt;&lt; 8 | Wire.read()) / 340.0 + 36.53;
  gyroX  = (Wire.read() &lt;&lt; 8 | Wire.read()) / 131.0;
  gyroY  = (Wire.read() &lt;&lt; 8 | Wire.read()) / 131.0;
  gyroZ  = (Wire.read() &lt;&lt; 8 | Wire.read()) / 131.0;

  Serial.print(&quot;Accel X: &quot;); Serial.print(accelX);
  Serial.print(&quot;  Y: &quot;);     Serial.print(accelY);
  Serial.print(&quot;  Z: &quot;);     Serial.println(accelZ);

  Serial.print(&quot;Gyro  X: &quot;); Serial.print(gyroX);
  Serial.print(&quot;  Y: &quot;);     Serial.print(gyroY);
  Serial.print(&quot;  Z: &quot;);     Serial.println(gyroZ);

  Serial.print(&quot;Temp: &quot;);    Serial.print(temp);
  Serial.println(&quot; C&quot;);
  Serial.println(&quot;---&quot;);

  delay(500);
}</code></pre>
</div>
</details>

---

## [`PumpCheck.ino`](../Arduino%20Code/PumpCheck/PumpCheck.ino)

Minimal pump relay test. Toggles the vacuum pump on and off via serial commands with no other hardware active. Used to isolate and debug pump-related brownout issues before the two-rail power redesign.

<details>
<summary>Arduino C | 52 lines</summary>
<div class="code-scroll">
<pre><code>// PickMasters — PumpCheck
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
  digitalWrite(PUMP_PIN, HIGH);   // active LOW -&gt; OFF at startup
  Serial.write(&quot;PickMasters pump test ready\n&quot;);
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == &#x27;\n&#x27;) {
      buf[len] = &#x27;\0&#x27;;
      handle(buf);
      len = 0;
    } else if (c != &#x27;\r&#x27; &amp;&amp; len &lt; sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

void handle(char *cmd) {
  if (strcmp(cmd, &quot;P1&quot;) == 0) {
    pumpOn = true;
    digitalWrite(PUMP_PIN, LOW);    // active LOW = ON
    Serial.write(&quot;pump ON\n&quot;);
  } else if (strcmp(cmd, &quot;P0&quot;) == 0) {
    pumpOn = false;
    digitalWrite(PUMP_PIN, HIGH);   // active LOW = OFF
    Serial.write(&quot;pump OFF\n&quot;);
  } else if (strcmp(cmd, &quot;status&quot;) == 0) {
    Serial.print(&quot;PUMP:&quot;);
    Serial.println(pumpOn ? &quot;ON&quot; : &quot;OFF&quot;);
  }
}</code></pre>
</div>
</details>

---

## [`PumpTest.ino`](../Arduino%20Code/PumpTest/PumpTest.ino)

Bare-minimum relay toggle. Even simpler than PumpCheck — just turns the relay on for a few seconds and off again. Used for initial hardware verification of the relay module wiring.

<details>
<summary>Arduino C | 28 lines</summary>
<div class="code-scroll">
<pre><code>const int RELAY_PIN = 7;

void setup() {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Pump off

  Serial.println(&quot;=== Pump Relay Test ===&quot;);
  Serial.println(&quot;&#x27;on&#x27;  - Turn pump on&quot;);
  Serial.println(&quot;&#x27;off&#x27; - Turn pump off&quot;);
  Serial.println(&quot;=======================\n&quot;);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil(&#x27;\n&#x27;);
    input.trim();

    if (input.equalsIgnoreCase(&quot;on&quot;)) {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println(&quot;Pump ON&quot;);
    }
    else if (input.equalsIgnoreCase(&quot;off&quot;)) {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println(&quot;Pump OFF&quot;);
    }
  }
}</code></pre>
</div>
</details>

---

## [`Full.ino`](../Arduino%20Code/Full/Full.ino)

An early Processing sketch that combined gamepad input with basic serial output. Uses the Firmata protocol to drive servos directly. Predates the separation into DriveControl.pde (manual) and detect_and_move.py (auto).

<details>
<summary>Processing (Java) | 90 lines</summary>
<div class="code-scroll">
<pre><code>import org.gamecontrolplus.*;
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
  cont = control.getMatchedDevice(&quot;Test4&quot;);

  if (cont == null) {
    println(&quot;Controller not connected&quot;);
    System.exit(-1);
  }

  println(Serial.list());
  port = new Serial(this, Serial.list()[1], 9600);
  port.bufferUntil(&#x27;\n&#x27;);

  delay(2000); // Wait for Arduino to boot
}

public void getUserInput() {
  float baseInput     = cont.getSlider(&quot;ServoBase&quot;).getValue();
  float shoulderInput = cont.getSlider(&quot;ServoShoulder&quot;).getValue();
  float elbowInput    = cont.getSlider(&quot;ServoElbow&quot;).getValue();

  if (abs(baseInput) &gt; deadzone)     baseAngle     += baseInput * speed;
  if (abs(shoulderInput) &gt; deadzone) shoulderAngle += shoulderInput * speed;
  if (abs(elbowInput) &gt; deadzone)    elbowAngle    += elbowInput * speed;

  baseAngle     = constrain(baseAngle, 0, 180);
  shoulderAngle = constrain(shoulderAngle, 0, 180);
  elbowAngle    = constrain(elbowAngle, 0, 180);
}

void sendCommand(int servo, int angle) {
  port.write(servo + &quot; &quot; + angle + &quot;\n&quot;);
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
  text(&quot;Base: &quot; + (int)baseAngle, 10, 30);
  text(&quot;Shoulder: &quot; + (int)shoulderAngle, 10, 50);
  text(&quot;Elbow: &quot; + (int)elbowAngle, 10, 70);
  text(&quot;Wrist: AUTO (IMU)&quot;, 10, 90);
}

// Print any feedback from Arduino
void serialEvent(Serial p) {
  String msg = p.readStringUntil(&#x27;\n&#x27;);
  if (msg != null) println(msg.trim());
}</code></pre>
</div>
</details>

---

## [`DriveControl.pde`](../Arduino%20Code/DriveControl/DriveControl.pde)

Development version of the manual-control gamepad interface. Full button mapping with D-pad, pump toggle, IMU calibration, and wrist AUTO/MANUAL mode switching. Functionally identical to the final version.

<details>
<summary>Processing (Java) | 231 lines</summary>
<div class="code-scroll">
<pre><code>// PickMasters — DriveControl (manual controller, paired with ArmController.ino)
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
//   CalButton             — IMU re-zero (&quot;cal&quot;)       (rising edge)
//   WristModeButton       — wrist AUTO/MANUAL toggle  (rising edge)
//
// Serial: ONE combined message per frame, sent only when something changed
// and at most every SEND_INTERVAL ms, to avoid flooding the Arduino:
//   S&lt;shoulder&gt;,&lt;elbow&gt;,&lt;wrist&gt;,&lt;hand&gt;,M&lt;left&gt;,&lt;right&gt;,P&lt;0|1&gt;,W&lt;0|1&gt;\n

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
String lastMsg  = &quot;&quot;;
long   lastSend = 0;
final int SEND_INTERVAL = 50;

String lastResponse = &quot;&quot;;

void setup() {
  size(440, 290);
  frameRate(50);

  // GCP on Windows enumerates every input device (including virtual ones like
  // FakerInput).  Wrapping init in try/catch lets us recover gracefully.
  try {
    control = ControlIO.getInstance(this);
    cont = control.getMatchedDevice(&quot;PickMasters&quot;);
  } catch (Exception e) {
    println(&quot;Warning during controller init: &quot; + e.getMessage());
  }

  if (cont == null) {
    println(&quot;Controller not found — check data/PickMasters config&quot;);
    System.exit(-1);
  }
  controllerReady = true;

  // Pick the Arduino&#x27;s serial port automatically: COM1 is almost always the
  // PC&#x27;s built-in port, so prefer the last port that isn&#x27;t COM1.  If the
  // Arduino is unplugged (or its driver is missing) no usable port exists.
  String[] ports = Serial.list();
  printArray(ports);

  String portName = null;
  for (int i = ports.length - 1; i &gt;= 0; i--) {
    if (!ports[i].equals(&quot;COM1&quot;)) { portName = ports[i]; break; }
  }
  if (portName == null &amp;&amp; ports.length &gt; 0) portName = ports[0];

  if (portName == null) {
    println(&quot;No serial port found — is the Arduino plugged in?&quot;);
    System.exit(-1);
  }

  println(&quot;Connecting to &quot; + portName);
  port = new Serial(this, portName, 9600);
  port.bufferUntil(&#x27;\n&#x27;);

  delay(2000);   // let the Arduino reboot after the port opens
}

void getUserInput() {
  if (!controllerReady || cont == null) return;
  float leftX  = cont.getSlider(&quot;LeftX&quot;).getValue();
  float leftY  = cont.getSlider(&quot;LeftY&quot;).getValue();
  float rightX = cont.getSlider(&quot;RightX&quot;).getValue();
  float rightY = cont.getSlider(&quot;RightY&quot;).getValue();

  // --- Base DC motors: digital only, single constant speed ---
  // Stick must be fully pushed (gamepads read negative Y when pushed forward).
  // Forward/back wins; left/right pivots in place (never mixed with fwd/back).
  if (leftY &lt;= -driveThreshold)      { motorLeft =  1; motorRight =  1; }  // forward
  else if (leftY &gt;= driveThreshold)  { motorLeft = -1; motorRight = -1; }  // backward
  else if (leftX &gt;= driveThreshold)  { motorLeft =  1; motorRight = -1; }  // pivot right
  else if (leftX &lt;= -driveThreshold) { motorLeft = -1; motorRight =  1; }  // pivot left
  else                               { motorLeft =  0; motorRight =  0; }

  // --- Shoulder &amp; elbow on the D-pad (fixed step per frame while held) ---
  // GameControlPlus hat positions: 0 = released, then clockwise from
  // 1 = up-left: 2 = up, 3 = up-right, 4 = right, 5 = down-right,
  // 6 = down, 7 = down-left, 8 = left.
  int pos = cont.getHat(&quot;Dpad&quot;).getPos();
  boolean dUp    = (pos == 1 || pos == 2 || pos == 3);
  boolean dDown  = (pos == 5 || pos == 6 || pos == 7);
  boolean dRight = (pos == 3 || pos == 4 || pos == 5);
  boolean dLeft  = (pos == 7 || pos == 8 || pos == 1);

  if (dUp)    shoulderAngle += dpadStep;
  if (dDown)  shoulderAngle -= dpadStep;
  if (dRight) elbowAngle    += dpadStep;
  if (dLeft)  elbowAngle    -= dpadStep;

  // --- Wrist (right stick Y, MANUAL only) &amp; hand (right stick X): DIGITAL ---
  // Like the drive sticks — the servo only moves at FULL deflection, stepping a
  // fixed amount per frame. No proportional/rate control: half-pushed does nothing.
  if (!wristAuto) {
    if (rightY &lt;= -driveThreshold)     wristAngle += stickStep;   // stick up   = wrist up
    else if (rightY &gt;= driveThreshold) wristAngle -= stickStep;   // stick down = wrist down
  }
  if (rightX &gt;= driveThreshold)        handAngle += stickStep;    // stick right = hand +
  else if (rightX &lt;= -driveThreshold)  handAngle -= stickStep;    // stick left  = hand -

  shoulderAngle = constrain(shoulderAngle, 0, 180);
  elbowAngle    = constrain(elbowAngle,    0, 180);
  wristAngle    = constrain(wristAngle,    0, 180);
  handAngle     = constrain(handAngle,     0, 180);

  // --- Buttons (rising edge only) ---
  boolean pumpBtn = cont.getButton(&quot;PumpButton&quot;).pressed();
  boolean calBtn  = cont.getButton(&quot;CalButton&quot;).pressed();
  boolean modeBtn = cont.getButton(&quot;WristModeButton&quot;).pressed();

  if (pumpBtn &amp;&amp; !prevPumpBtn) pumpOn = !pumpOn;
  if (modeBtn &amp;&amp; !prevModeBtn) wristAuto = !wristAuto;
  if (calBtn &amp;&amp; !prevCalBtn)   port.write(&quot;cal\n&quot;);

  prevPumpBtn = pumpBtn;
  prevCalBtn  = calBtn;
  prevModeBtn = modeBtn;
}

void sendState() {
  // ONE combined message per frame — only when it changed, throttled to
  // SEND_INTERVAL, written with port.write() (no println), to keep the
  // Arduino&#x27;s serial buffer from overflowing and dropping the connection.
  String msg = &quot;S&quot; + (int)shoulderAngle + &quot;,&quot; + (int)elbowAngle + &quot;,&quot;
                   + (int)wristAngle + &quot;,&quot; + (int)handAngle
             + &quot;,M&quot; + motorLeft + &quot;,&quot; + motorRight
             + &quot;,P&quot; + (pumpOn ? 1 : 0)
             + &quot;,W&quot; + (wristAuto ? 1 : 0) + &quot;\n&quot;;

  if (!msg.equals(lastMsg) &amp;&amp; millis() - lastSend &gt;= SEND_INTERVAL) {
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
    // skip this frame&#x27;s input, will re-read next frame
  }
  sendState();

  background(40, 60, 100);

  fill(255);
  textSize(16);
  text(&quot;PickMasters  —  Manual Mode&quot;, 10, 28);

  textSize(14);
  fill(200, 230, 255);
  text(&quot;Shoulder:  &quot; + (int)shoulderAngle + &quot; deg&quot;, 10, 60);
  text(&quot;Elbow:     &quot; + (int)elbowAngle + &quot; deg&quot;, 10, 82);
  text(&quot;Wrist:     &quot; + (int)wristAngle + &quot; deg&quot;, 10, 104);
  text(&quot;Hand:      &quot; + (int)handAngle + &quot; deg&quot;, 10, 126);
  text(&quot;Motors:    L &quot; + motorState(motorLeft) + &quot;   R &quot; + motorState(motorRight), 10, 148);

  fill(pumpOn ? color(80, 255, 80) : color(255, 80, 80));
  text(&quot;Pump:      &quot; + (pumpOn ? &quot;ON&quot; : &quot;OFF&quot;), 10, 170);

  fill(wristAuto ? color(120, 200, 255) : color(255, 200, 80));
  text(&quot;Wrist mode: &quot; + (wristAuto ? &quot;AUTO (IMU)&quot; : &quot;MANUAL (right stick Y)&quot;), 10, 192);

  fill(160);
  textSize(11);
  text(&quot;Left stick: drive (full push)   D-pad: shoulder/elbow   Right stick: wrist/hand&quot;, 10, 230);
  text(&quot;PumpButton: pump   CalButton: IMU re-zero   WristModeButton: AUTO/MANUAL&quot;, 10, 247);
  fill(220);
  text(&quot;Arduino: &quot; + lastResponse, 10, 275);
}

String motorState(int dir) {
  if (dir &gt; 0) return &quot;FWD&quot;;
  if (dir &lt; 0) return &quot;REV&quot;;
  return &quot;STOP&quot;;
}

void serialEvent(Serial p) {
  String msg = p.readStringUntil(&#x27;\n&#x27;);
  if (msg != null) {
    lastResponse = msg.trim();
    println(lastResponse);
  }
}</code></pre>
</div>
</details>

---

## [`ManualMode.pde`](../Arduino%20Code/ManualMode/ManualMode.pde)

An earlier, simpler Processing sketch for manual control. Three-axis gamepad mapping with no serial throttling — replaced by the full DriveControl once the control mapping was finalized.

<details>
<summary>Processing (Java) | 90 lines</summary>
<div class="code-scroll">
<pre><code>import org.gamecontrolplus.*;
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
  cont = control.getMatchedDevice(&quot;Test4&quot;);

  if (cont == null) {
    println(&quot;Controller not connected&quot;);
    System.exit(-1);
  }

  println(Serial.list());
  port = new Serial(this, Serial.list()[1], 9600);
  port.bufferUntil(&#x27;\n&#x27;);

  delay(2000); // Wait for Arduino to boot
}

public void getUserInput() {
  float baseInput     = cont.getSlider(&quot;ServoBase&quot;).getValue();
  float shoulderInput = cont.getSlider(&quot;ServoShoulder&quot;).getValue();
  float elbowInput    = cont.getSlider(&quot;ServoElbow&quot;).getValue();

  if (abs(baseInput) &gt; deadzone)     baseAngle     += baseInput * speed;
  if (abs(shoulderInput) &gt; deadzone) shoulderAngle += shoulderInput * speed;
  if (abs(elbowInput) &gt; deadzone)    elbowAngle    += elbowInput * speed;

  baseAngle     = constrain(baseAngle, 0, 180);
  shoulderAngle = constrain(shoulderAngle, 0, 180);
  elbowAngle    = constrain(elbowAngle, 0, 180);
}

void sendCommand(int servo, int angle) {
  port.write(servo + &quot; &quot; + angle + &quot;\n&quot;);
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
  text(&quot;Base: &quot; + (int)baseAngle, 10, 30);
  text(&quot;Shoulder: &quot; + (int)shoulderAngle, 10, 50);
  text(&quot;Elbow: &quot; + (int)elbowAngle, 10, 70);
  text(&quot;Wrist: AUTO (IMU)&quot;, 10, 90);
}

// Print any feedback from Arduino
void serialEvent(Serial p) {
  String msg = p.readStringUntil(&#x27;\n&#x27;);
  if (msg != null) println(msg.trim());
}</code></pre>
</div>
</details>

---

## [`PumpCheckDrive.pde`](../Arduino%20Code/PumpCheckDrive/PumpCheckDrive.pde)

Processing companion for <code>PumpCheck.ino</code>. Adds gamepad-based pump toggle to verify the full input chain: gamepad → Processing → serial → Arduino → relay → pump.

<details>
<summary>Processing (Java) | 99 lines</summary>
<div class="code-scroll">
<pre><code>// PickMasters — PumpCheckDrive (controller test, paired with PumpCheck.ino)
//
// Minimal Processing sketch: reads ONLY the pump button on the Bluetooth
// gamepad (GameControlPlus, config: data/PickMasters) and toggles the pump on
// the Arduino. Use this to confirm the whole chain works:
//   gamepad button -&gt; serial -&gt; Arduino -&gt; pump relay.
//
// Serial: &quot;P1\n&quot; = pump on, &quot;P0\n&quot; = pump off (9600 baud).

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
String  lastResponse = &quot;&quot;;

void setup() {
  size(380, 170);
  frameRate(50);

  // Match the controller (same config file as the full DriveControl sketch).
  try {
    control = ControlIO.getInstance(this);
    cont = control.getMatchedDevice(&quot;PickMasters&quot;);
  } catch (Exception e) {
    println(&quot;Warning during controller init: &quot; + e.getMessage());
  }
  if (cont == null) {
    println(&quot;Controller not found — check data/PickMasters config&quot;);
    System.exit(-1);
  }
  controllerReady = true;

  // Pick the Arduino&#x27;s serial port: prefer the last port that isn&#x27;t COM1.
  String[] ports = Serial.list();
  printArray(ports);

  String portName = null;
  for (int i = ports.length - 1; i &gt;= 0; i--) {
    if (!ports[i].equals(&quot;COM1&quot;)) { portName = ports[i]; break; }
  }
  if (portName == null &amp;&amp; ports.length &gt; 0) portName = ports[0];
  if (portName == null) {
    println(&quot;No serial port found — is the Arduino plugged in?&quot;);
    System.exit(-1);
  }

  println(&quot;Connecting to &quot; + portName);
  port = new Serial(this, portName, 9600);
  port.bufferUntil(&#x27;\n&#x27;);
  delay(2000);   // let the Arduino reboot after the port opens
}

void draw() {
  // ConcurrentModificationException is a known GCP bug — skip the frame.
  try {
    if (controllerReady &amp;&amp; cont != null) {
      boolean pumpBtn = cont.getButton(&quot;PumpButton&quot;).pressed();
      if (pumpBtn &amp;&amp; !prevPumpBtn) {          // rising edge = press
        pumpOn = !pumpOn;
        port.write(pumpOn ? &quot;P1\n&quot; : &quot;P0\n&quot;);
      }
      prevPumpBtn = pumpBtn;
    }
  } catch (java.util.ConcurrentModificationException e) {
    // skip this frame&#x27;s input
  }

  background(40, 60, 100);
  fill(255);
  textSize(16);
  text(&quot;PickMasters  —  Pump Test&quot;, 10, 28);

  fill(pumpOn ? color(80, 255, 80) : color(255, 80, 80));
  textSize(22);
  text(&quot;PUMP: &quot; + (pumpOn ? &quot;ON&quot; : &quot;OFF&quot;), 10, 72);

  fill(200);
  textSize(12);
  text(&quot;Press the PumpButton on the controller to toggle.&quot;, 10, 105);
  fill(220);
  text(&quot;Arduino: &quot; + lastResponse, 10, 140);
}

void serialEvent(Serial p) {
  String msg = p.readStringUntil(&#x27;\n&#x27;);
  if (msg != null) {
    lastResponse = msg.trim();
    println(lastResponse);
  }
}</code></pre>
</div>
</details>

---

## [`School.pde`](../Arduino%20Code/School/School.pde)

Minimal Processing test sketch used during early lab sessions. Basic gamepad-to-servo serial communication using the Firmata protocol.

<details>
<summary>Processing (Java) | 69 lines</summary>
<div class="code-scroll">
<pre><code>import org.gamecontrolplus.*;
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
  cont = control.getMatchedDevice(&quot;Test4&quot;);

  if (cont == null) {
    println(&quot;Not connected&quot;);
    System.exit(-1);
  }

  arduino = new Arduino(this, Arduino.list()[1], 57600);
  arduino.pinMode(9, Arduino.SERVO);
  arduino.pinMode(10, Arduino.SERVO);
  arduino.pinMode(11, Arduino.SERVO);
  arduino.pinMode(12, Arduino.SERVO);
}

public void getUserInput() {
  float baseInput     = cont.getSlider(&quot;ServoBase&quot;).getValue();
  float shoulderInput = cont.getSlider(&quot;ServoShoulder&quot;).getValue();
  float elbowInput    = cont.getSlider(&quot;ServoElbow&quot;).getValue();
  float wristInput    = cont.getSlider(&quot;ServoWrist&quot;).getValue();

  // Apply deadzone
  if (abs(baseInput) &gt; deadzone)     baseAngle     += baseInput * speed;
  if (abs(shoulderInput) &gt; deadzone) shoulderAngle += shoulderInput * speed;
  if (abs(elbowInput) &gt; deadzone)    elbowAngle    += elbowInput * speed;
  if (abs(wristInput) &gt; deadzone)    wristAngle    += wristInput * speed;

  // Clamp to 0-180
  baseAngle     = constrain(baseAngle, 0, 180);
  shoulderAngle = constrain(shoulderAngle, 0, 180);
  elbowAngle    = constrain(elbowAngle, 0, 180);
  wristAngle    = constrain(wristAngle, 0, 180);

  println(&quot;Base: &quot; + baseAngle + &quot;  Shoulder: &quot; + shoulderAngle +
          &quot;  Elbow: &quot; + elbowAngle + &quot;  Wrist: &quot; + wristAngle);
}

void draw() {
  getUserInput();
  background(baseAngle, shoulderAngle, 255);

  arduino.servoWrite(9, (int)baseAngle);
  arduino.servoWrite(10, (int)shoulderAngle);
  arduino.servoWrite(11, (int)elbowAngle);
  arduino.servoWrite(12, (int)wristAngle);
}</code></pre>
</div>
</details>

---

[← Back to Home](../index.md)
