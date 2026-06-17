// PickMasters — PumpCheck
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
  digitalWrite(PUMP_PIN, HIGH);   // active LOW -> OFF at startup
  Serial.write("PickMasters pump test ready\n");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      buf[len] = '\0';
      handle(buf);
      len = 0;
    } else if (c != '\r' && len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

void handle(char *cmd) {
  if (strcmp(cmd, "P1") == 0) {
    pumpOn = true;
    digitalWrite(PUMP_PIN, LOW);    // active LOW = ON
    Serial.write("pump ON\n");
  } else if (strcmp(cmd, "P0") == 0) {
    pumpOn = false;
    digitalWrite(PUMP_PIN, HIGH);   // active LOW = OFF
    Serial.write("pump OFF\n");
  } else if (strcmp(cmd, "status") == 0) {
    Serial.print("PUMP:");
    Serial.println(pumpOn ? "ON" : "OFF");
  }
}
