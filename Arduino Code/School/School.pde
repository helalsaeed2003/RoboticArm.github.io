import org.gamecontrolplus.*;
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
  cont = control.getMatchedDevice("Test4");

  if (cont == null) {
    println("Not connected");
    System.exit(-1);
  }

  arduino = new Arduino(this, Arduino.list()[1], 57600);
  arduino.pinMode(9, Arduino.SERVO);
  arduino.pinMode(10, Arduino.SERVO);
  arduino.pinMode(11, Arduino.SERVO);
  arduino.pinMode(12, Arduino.SERVO);
}

public void getUserInput() {
  float baseInput     = cont.getSlider("ServoBase").getValue();
  float shoulderInput = cont.getSlider("ServoShoulder").getValue();
  float elbowInput    = cont.getSlider("ServoElbow").getValue();
  float wristInput    = cont.getSlider("ServoWrist").getValue();

  // Apply deadzone
  if (abs(baseInput) > deadzone)     baseAngle     += baseInput * speed;
  if (abs(shoulderInput) > deadzone) shoulderAngle += shoulderInput * speed;
  if (abs(elbowInput) > deadzone)    elbowAngle    += elbowInput * speed;
  if (abs(wristInput) > deadzone)    wristAngle    += wristInput * speed;

  // Clamp to 0-180
  baseAngle     = constrain(baseAngle, 0, 180);
  shoulderAngle = constrain(shoulderAngle, 0, 180);
  elbowAngle    = constrain(elbowAngle, 0, 180);
  wristAngle    = constrain(wristAngle, 0, 180);

  println("Base: " + baseAngle + "  Shoulder: " + shoulderAngle +
          "  Elbow: " + elbowAngle + "  Wrist: " + wristAngle);
}

void draw() {
  getUserInput();
  background(baseAngle, shoulderAngle, 255);

  arduino.servoWrite(9, (int)baseAngle);
  arduino.servoWrite(10, (int)shoulderAngle);
  arduino.servoWrite(11, (int)elbowAngle);
  arduino.servoWrite(12, (int)wristAngle);
}
