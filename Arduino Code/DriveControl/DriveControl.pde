import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;
import processing.serial.*;
import net.java.games.input.*;

ControlDevice cont;
ControlIO control;
Serial port;

float shoulderAngle = 90;
float elbowAngle    = 90;

float speed    = 4.0;
float deadzone = 0.2;

int prevShoulder   = -1;
int prevElbow      = -1;
int prevLeftSpeed  = 1;
int prevRightSpeed = 1;

boolean pumpOn         = false;
boolean prevPumpButton = false;
boolean pumpChanged    = true;

void setup() {
  size(420, 220);
  frameRate(50);

  control = ControlIO.getInstance(this);
  cont = control.getMatchedDevice("CompleteControl");

  if (cont == null) {
    println("Controller not found — check data/CompleteControl config");
    System.exit(-1);
  }

  println(Serial.list());
  port = new Serial(this, Serial.list()[1], 9600);
  port.bufferUntil('\n');

  delay(2000);
}

void getUserInput() {
  // Left stick — base DC motors
  // Each direction is a separate named slider (0..1 when active, config handles sign flip)
  float fwdVal   = max(0, cont.getSlider("Forward").getValue());
  float bwdVal   = max(0, cont.getSlider("Backward").getValue());
  float leftVal  = max(0, cont.getSlider("TurnLeft").getValue());
  float rightVal = max(0, cont.getSlider("TurnRight").getValue());

  // Right stick — arm servos
  float rightX = cont.getSlider("RightX").getValue();
  float rightY = cont.getSlider("RightY").getValue();

  // Button — pump toggle
  boolean btn = cont.getButton("PumpButton").pressed();

  // --- Differential drive ---
  float fwd  = (fwdVal  > deadzone ? fwdVal  : 0) - (bwdVal  > deadzone ? bwdVal  : 0);
  float turn = (rightVal > deadzone ? rightVal : 0) - (leftVal > deadzone ? leftVal : 0);

  int leftSpeed  = (int)(constrain(fwd + turn, -1.0, 1.0) * 255);
  int rightSpeed = (int)(constrain(fwd - turn, -1.0, 1.0) * 255);

  // --- Shoulder & Elbow ---
  if (abs(rightY) > deadzone) shoulderAngle -= rightY * speed;
  if (abs(rightX) > deadzone) elbowAngle    += rightX * speed;

  shoulderAngle = constrain(shoulderAngle, 0, 180);
  elbowAngle    = constrain(elbowAngle,    0, 180);

  // --- Pump toggle (rising edge) ---
  if (btn && !prevPumpButton) {
    pumpOn      = !pumpOn;
    pumpChanged = true;
  }
  prevPumpButton = btn;

  // --- Send only on change ---
  if (leftSpeed != prevLeftSpeed || rightSpeed != prevRightSpeed) {
    port.write("M " + leftSpeed + " " + rightSpeed + "\n");
    prevLeftSpeed  = leftSpeed;
    prevRightSpeed = rightSpeed;
  }

  if ((int)shoulderAngle != prevShoulder) {
    port.write("2 " + (int)shoulderAngle + "\n");
    prevShoulder = (int)shoulderAngle;
  }

  if ((int)elbowAngle != prevElbow) {
    port.write("3 " + (int)elbowAngle + "\n");
    prevElbow = (int)elbowAngle;
  }

  if (pumpChanged) {
    port.write("P " + (pumpOn ? "1" : "0") + "\n");
    pumpChanged = false;
  }
}

void draw() {
  getUserInput();
  background(40, 60, 100);

  fill(255);
  textSize(16);
  text("PickMasters  —  Drive Control", 10, 28);

  textSize(14);
  fill(200, 230, 255);
  text("Base:      DC motors  (left stick)", 10, 60);
  text("Shoulder:  " + (int)shoulderAngle + " deg", 10, 82);
  text("Elbow:     " + (int)elbowAngle + " deg", 10, 104);
  text("Wrist:     AUTO (IMU)", 10, 126);

  fill(pumpOn ? color(80, 255, 80) : color(255, 80, 80));
  text("Pump:      " + (pumpOn ? "ON" : "OFF"), 10, 148);

  fill(160);
  textSize(11);
  text("Left stick: drive/turn   Right stick: shoulder/elbow   Button 0: pump toggle", 10, 185);
  text("IMU auto-levels wrist — send 'cal' in serial monitor to re-zero", 10, 202);
}

void serialEvent(Serial p) {
  String msg = p.readStringUntil('\n');
  if (msg != null) println(msg.trim());
}
