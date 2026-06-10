// PickMasters — DriveControl (manual controller, paired with ArmController.ino)
//
// Bluetooth XINPUT gamepad via GameControlPlus (config: data/PickMasters).
//
// Mapping:
//   Left stick (digital)  — base drive: fully forward/back = both wheels,
//                           fully left/right = pivot in place. Constant speed.
//   D-pad up/down         — shoulder servo  (±step per frame while held)
//   D-pad left/right      — elbow servo     (±step per frame while held)
//   Right stick Y         — wrist servo     (rate control, MANUAL mode only)
//   Right stick X         — hand servo      (rate control)
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
float stickRate = 3.0;   // deg per frame at full deflection (wrist/hand)
float deadzone  = 0.2;   // right stick deadzone

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

  println(Serial.list());
  port = new Serial(this, Serial.list()[1], 9600);
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

  // --- Wrist (right stick Y, MANUAL mode only) & hand (right stick X) ---
  if (!wristAuto && abs(rightY) > deadzone) wristAngle -= rightY * stickRate;
  if (abs(rightX) > deadzone)               handAngle  += rightX * stickRate;

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
