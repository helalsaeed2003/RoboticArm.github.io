import org.gamecontrolplus.*;
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
  cont = control.getMatchedDevice("Test4");

  if (cont == null) {
    println("Controller not connected");
    System.exit(-1);
  }

  println(Serial.list());
  port = new Serial(this, Serial.list()[1], 9600);
  port.bufferUntil('\n');

  delay(2000); // Wait for Arduino to boot
}

public void getUserInput() {
  float baseInput     = cont.getSlider("ServoBase").getValue();
  float shoulderInput = cont.getSlider("ServoShoulder").getValue();
  float elbowInput    = cont.getSlider("ServoElbow").getValue();

  if (abs(baseInput) > deadzone)     baseAngle     += baseInput * speed;
  if (abs(shoulderInput) > deadzone) shoulderAngle += shoulderInput * speed;
  if (abs(elbowInput) > deadzone)    elbowAngle    += elbowInput * speed;

  baseAngle     = constrain(baseAngle, 0, 180);
  shoulderAngle = constrain(shoulderAngle, 0, 180);
  elbowAngle    = constrain(elbowAngle, 0, 180);
}

void sendCommand(int servo, int angle) {
  port.write(servo + " " + angle + "\n");
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
  text("Base: " + (int)baseAngle, 10, 30);
  text("Shoulder: " + (int)shoulderAngle, 10, 50);
  text("Elbow: " + (int)elbowAngle, 10, 70);
  text("Wrist: AUTO (IMU)", 10, 90);
}

// Print any feedback from Arduino
void serialEvent(Serial p) {
  String msg = p.readStringUntil('\n');
  if (msg != null) println(msg.trim());
}
