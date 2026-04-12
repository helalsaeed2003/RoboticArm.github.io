#include <Servo.h>

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

  Serial.println("=== 2-DOF Servo Controller ===");
  Serial.println("Format:  <servo> <angle>");
  Serial.println("  servo : 1=Base  2=Shoulder");
  Serial.println("  angle : 0 - 270");
  Serial.println("Example: 2 45   (moves Shoulder to 45 deg)");
  Serial.println("Type 'status' to see current angles.");
  Serial.println("==============================\n");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("status")) {
      printStatus();
      return;
    }

    int spaceIndex = input.indexOf(' ');
    if (spaceIndex == -1) {
      Serial.println("Invalid format. Use: <servo> <angle>");
      return;
    }

    int servoNum = input.substring(0, spaceIndex).toInt();
    int angle    = input.substring(spaceIndex + 1).toInt();

    if (servoNum < 1 || servoNum > 2) {
      Serial.println("Servo must be 1 or 2.");
      return;
    }
    if (angle < 0 || angle > 270) {
      Serial.println("Angle must be 0-270.");
      return;
    }

    switch (servoNum) {
      case 1:
        baseAngle = angle;
        baseServo.write(angle);
        Serial.print("Base -> ");
        break;
      case 2:
        shoulderAngle = angle;
        shoulderServo.write(angle);
        Serial.print("Shoulder -> ");
        break;
    }
    Serial.print(angle);
    Serial.println(" deg");
  }
}

void printStatus() {
  Serial.println("----- Current Angles -----");
  Serial.print("  1) Base     : "); Serial.println(baseAngle);
  Serial.print("  2) Shoulder : "); Serial.println(shoulderAngle);
  Serial.println("--------------------------");
}