# Embedded Control Logic and Programming

[← Back to Home](../index.md)

---

## Two-Sketch Firmware Architecture

Two Arduino sketches, one per mode. **ArmController.ino** (manual mode) drives all four arm servos, both DC motors, the pump relay, the IMU loop, and the safety interlock. **PickAndMove.ino** (automatic mode) drives only the two DC base motors via the fuzzy-logic controller — arm servos are inactive in auto mode and the operator switches back to manual for the pick step. Two sketches give clear responsibility per file and let the two control laws (PID on the wrist; fuzzy on the base) be explained separately. Mode switching requires reflashing — acceptable for the benchtop demo, would change for industrial use. Pin allocation for L298N, MPU6050, pump relay, and safety input is shared between both sketches.

## Arm Controller Firmware — ArmController.ino

Written in C. Main loop runs at full ATmega328P speed; internal timing gates IMU/wrist updates at 50 Hz. Every pass: refresh watchdog → handle pending serial → update safety interlock → if clear and IMU is due, read IMU and update wrist. Organised into six modules: serial parser with XOR checksum, IMU driver, PID wrist controller, safety-interlock state machine, DC motor driver layer over L298N, watchdog supervisor. Full source: Appendix C.1.

## Arm Serial Protocol with Parity Checking

USB serial, 9600 baud. One line-oriented message sent only when a field changes (~50 ms cadence):

```
S<shoulder>,<elbow>,<wrist>,<hand>,M<left>,<right>,P<0|1>,W<0|1>*<XX>
```

Angles 0–180; motor directions in {−1, 0, +1}; P = pump on/off; W = wrist auto (1) or manual (0). The trailing `*XX` is an optional XOR checksum of all bytes before the asterisk. Mismatched checksums are rejected and an error returned. Commands without `*XX` are accepted for terminal diagnostics.

## Safety Interlock — Sequential Logic State Machine

A single boolean `safetyClear` gates the actuators — if low, pump and motors are forced safe regardless of last received command. Servos remain controllable (mechanically self-limited). `safetyClear` is recomputed every 50 ms as the AND of three conditions:

1. Hardware interlock pin A0 is HIGH (external pull-down to a normally-open e-stop).
2. Serial link is alive — no command received in > 2 s → assume host disconnected.
3. Sensors healthy — IMU must respond on I²C at boot, or auto-levelling is disabled.

Rising edge emits a confirmation over serial; falling edge emits a fault and immediately disables actuators.

## Hardware Watchdog Timer

AVR watchdog enabled at 2 s in `setup()` (`wdt_enable(WDTO_2S)`), reset every loop pass (`wdt_reset()`). A hung block (e.g. I²C wiring fault) triggers a reset back to the safe state. I²C library is also given a 3 ms bus timeout for graceful recovery from transient bus glitches.

## Wrist Auto-Levelling — PID Loop

PID compensator [1, Ch. 21]. Setpoint = 0° (level). Process variable = pitch from atan2 of accelerometer X and Z. Output drives the wrist servo around 90°. Gains tuned empirically: **Kp = 1.0, Ki = 0.05, Kd = 0.15.** Two safeguards: integral term clamped to ±30°·s (anti-windup); total output saturated to 0–180° servo range. Loop runs at 50 Hz — well above the MG996R's ~10 Hz internal bandwidth, satisfying Nyquist.

## Pick-and-Move Controller — Fuzzy Logic Vision-Guided Motion

PickAndMove.ino uses a Mamdani fuzzy controller. Input: absolute pixel error between target object centre and target-box centre. Output: motor PWM speed. Fuzzy is used because the pixel-error → speed relationship is non-linear (aggressive when far, gentle when close) — no single proportional gain works for both regimes.

- **Input membership functions (trapezoidal):** Small (0, 20, 40, 60); Medium (40, 60, 80, 120, 160); Large (120, 160, 200, ∞, ∞).
- **Output singletons (PWM 0–255):** Slow = 80, Medium = 150, Fast = 230.
- **Rules:** IF error Small → speed Slow; IF Medium → Medium; IF Large → Fast.
- **Defuzzification:** centre-of-gravity over the singletons weighted by firing strength.
- **Dead-zone fallback:** below a small firing-strength threshold, return a 60-PWM creep speed.

Motion is pulse-based: motors run 70 ms then auto-stop; the host re-sends pulses as long as motion is needed. A dropped serial link, host crash, or any communication failure halts the robot — a dead-man's-switch [1, Sec. 5.4]. YOLOv11 supplies the perception side [1, Ch. 22].

## Choice of Programming Language — Assembly vs C

C used for both sketches. Assembly was considered for I²C IMU reads and the inner fuzzy defuzzification loop. Rejected both: I²C reads complete in well under the 20 ms loop budget; software-float defuzzification takes under 1 ms. No measurable timing benefit, and the legibility cost of mixed-language code is high. Standard guidance from [1, Ch. 12] for modern 8-bit MCUs.

## Host Computer Software

Two host programs, one per mode. Manual: Processing sketch (Appendix C.3) using GameControlPlus; reads the Bluetooth gamepad, applies the XOR checksum, sends a frame at most every 50 ms only if a field changed. Automatic: `detect_and_move.py` (Appendix C.4) using OpenCV + YOLOv11; locks onto the highest-confidence detection above 50%, computes signed pixel errors, and sends FUZZY_PIVOT / FUZZY_DRIVE / STOP commands.

## Distributed Control Architecture

Current architecture is a degenerate distributed system [1, Sec. 15.2]: host runs high-level logic, Arduino runs low-level real-time control, single USB serial link between them. Adequate for the benchtop demo, not assembly-line ready. Three extensions considered for industrial use: CAN bus for noise immunity and direct conveyor coordination; Modbus RTU layered over the existing USB serial; ROS 2 topic over Ethernet (the ROS 2 twin in §10.4 is already subscriber-ready). None are in the prototype.

## Industrial Transition Note

A real deployment would replace the Arduino with a PLC [1, Ch. 14] for hardened I/O, noise immunity, and reliability. The watchdog and safety interlock would map to standard PLC equivalents; the XOR checksum would fold into the fieldbus error detection. The Arduino is fine for the prototype but would not survive production.

---

[← Back to Home](../index.md)
