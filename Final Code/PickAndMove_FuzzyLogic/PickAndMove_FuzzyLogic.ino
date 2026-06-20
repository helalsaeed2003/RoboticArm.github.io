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
