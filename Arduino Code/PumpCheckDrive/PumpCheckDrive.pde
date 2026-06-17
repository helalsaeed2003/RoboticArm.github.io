// PickMasters — PumpCheckDrive (controller test, paired with PumpCheck.ino)
//
// Minimal Processing sketch: reads ONLY the pump button on the Bluetooth
// gamepad (GameControlPlus, config: data/PickMasters) and toggles the pump on
// the Arduino. Use this to confirm the whole chain works:
//   gamepad button -> serial -> Arduino -> pump relay.
//
// Serial: "P1\n" = pump on, "P0\n" = pump off (9600 baud).

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
String  lastResponse = "";

void setup() {
  size(380, 170);
  frameRate(50);

  // Match the controller (same config file as the full DriveControl sketch).
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

  // Pick the Arduino's serial port: prefer the last port that isn't COM1.
  String[] ports = Serial.list();
  printArray(ports);

  String portName = null;
  for (int i = ports.length - 1; i >= 0; i--) {
    if (!ports[i].equals("COM1")) { portName = ports[i]; break; }
  }
  if (portName == null && ports.length > 0) portName = ports[0];
  if (portName == null) {
    println("No serial port found — is the Arduino plugged in?");
    System.exit(-1);
  }

  println("Connecting to " + portName);
  port = new Serial(this, portName, 9600);
  port.bufferUntil('\n');
  delay(2000);   // let the Arduino reboot after the port opens
}

void draw() {
  // ConcurrentModificationException is a known GCP bug — skip the frame.
  try {
    if (controllerReady && cont != null) {
      boolean pumpBtn = cont.getButton("PumpButton").pressed();
      if (pumpBtn && !prevPumpBtn) {          // rising edge = press
        pumpOn = !pumpOn;
        port.write(pumpOn ? "P1\n" : "P0\n");
      }
      prevPumpBtn = pumpBtn;
    }
  } catch (java.util.ConcurrentModificationException e) {
    // skip this frame's input
  }

  background(40, 60, 100);
  fill(255);
  textSize(16);
  text("PickMasters  —  Pump Test", 10, 28);

  fill(pumpOn ? color(80, 255, 80) : color(255, 80, 80));
  textSize(22);
  text("PUMP: " + (pumpOn ? "ON" : "OFF"), 10, 72);

  fill(200);
  textSize(12);
  text("Press the PumpButton on the controller to toggle.", 10, 105);
  fill(220);
  text("Arduino: " + lastResponse, 10, 140);
}

void serialEvent(Serial p) {
  String msg = p.readStringUntil('\n');
  if (msg != null) {
    lastResponse = msg.trim();
    println(lastResponse);
  }
}
