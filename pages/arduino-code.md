# Development Sketches

[← Back to Home](../index.md)

---

These are the development-stage Arduino and Processing sketches created during the build process. They test individual components, iterate on control approaches, and document the evolution from basic servo tests to the final integrated firmware. The production code is in [Final Code](final-code.md).

---

## `ArmController.ino`

> Arduino C | 267 lines

An earlier version of the manual-mode firmware before PID wrist levelling and the safety interlock were added. Drives servos and DC motors via serial commands from the Processing gamepad sketch. Used during mid-project integration testing.

---

## `ComponentTest.ino`

> Arduino C | 251 lines

Interactive bench-test sketch for every hardware component. Open the Serial Monitor, type a command, and individually test each servo, DC motor direction, pump relay, and IMU reading. Essential for verifying wiring before running the full firmware.

---

## `AutoMode.ino`

> Arduino C | 184 lines

Early automatic-mode prototype. Combines servo control with basic serial commands from the vision script. Predates the fuzzy-logic controller — uses fixed-speed motor commands instead.

---

## `CameraVisionTest.ino`

> Arduino C | 203 lines

Test firmware for validating the camera-to-Arduino communication loop. Accepts vision commands over serial and drives servos and motors in response. Used to debug the serial protocol between `detect_and_move.py` and the Arduino.

---

## `ManualDrive.ino`

> Arduino C | 224 lines

The original manual-drive firmware written for the L293D motor shield (before it shorted). Uses the AFMotor library. Kept in the repo as a record of the original hardware design and the lesson learned about driver current ratings.

---

## `SelfLevling.ino`

> Arduino C | 100 lines

Standalone wrist auto-levelling test. Reads the MPU6050 over I²C, computes pitch from accelerometer data, and drives a single servo to keep the wrist level. Used to tune the PID gains before integrating into the full ArmController.

---

## `PickAndMove.ino`

> Arduino C | 131 lines

Earlier version of the automatic-mode firmware. Simpler command set without the fuzzy-logic speed controller — uses fixed PWM values for each direction command.

---

## `ServoTest.ino`

> Arduino C | 79 lines

Minimal sketch to sweep all four servos through their range. Used to verify servo wiring, confirm PWM pin assignments, and check for mechanical binding in the 3D-printed joints.

---

## `SesnsorTest.ino`

> Arduino C | 66 lines

IMU sensor test. Reads raw accelerometer and gyroscope values from the MPU6050 and prints them to the Serial Monitor. Used to verify I²C communication and check sensor orientation before writing the pitch calculation.

---

## `PumpCheck.ino`

> Arduino C | 52 lines

Minimal pump relay test. Toggles the vacuum pump on and off via serial commands with no other hardware active. Used to isolate and debug pump-related brownout issues before the two-rail power redesign.

---

## `PumpTest.ino`

> Arduino C | 27 lines

Bare-minimum relay toggle. Even simpler than PumpCheck — just turns the relay on for a few seconds and off again. Used for initial hardware verification of the relay module wiring.

---

## `Full.ino`

> Processing (Java) | 89 lines

An early Processing sketch that combined gamepad input with basic serial output. Predates the separation into DriveControl.pde (manual) and detect_and_move.py (auto).

---

## `DriveControl.pde`

> Processing (Java) | 231 lines

Development version of the manual-control gamepad interface. Functionally identical to the final version but used during iterative testing with the earlier ArmController firmware.

---

## `ManualMode.pde`

> Processing (Java) | 90 lines

An earlier, simpler Processing sketch for manual control. Fewer axes mapped and no serial throttling — replaced by the full DriveControl once the control mapping was finalized.

---

## `PumpCheckDrive.pde`

> Processing (Java) | 99 lines

Processing companion for `PumpCheck.ino`. Adds gamepad-based pump toggle to verify the full input chain: gamepad → Processing → serial → Arduino → relay → pump.

---

## `School.pde`

> Processing (Java) | 69 lines

Minimal Processing test sketch used during early lab sessions. Basic serial communication test with a simple UI.

---

[← Back to Home](../index.md)
