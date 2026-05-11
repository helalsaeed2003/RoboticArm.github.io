const int RELAY_PIN = 7;

void setup() {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Pump off

  Serial.println("=== Pump Relay Test ===");
  Serial.println("'on'  - Turn pump on");
  Serial.println("'off' - Turn pump off");
  Serial.println("=======================\n");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("on")) {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Pump ON");
    }
    else if (input.equalsIgnoreCase("off")) {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Pump OFF");
    }
  }
}