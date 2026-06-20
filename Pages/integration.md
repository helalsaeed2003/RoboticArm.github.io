# Final Integration and Demonstration

[← Back to Home](../index.md)

---

## Testing Environment

- Controlled tabletop workspace with known lighting conditions
- Objects of distinct colors and simple geometric shapes (circles, squares, triangles)
- USB camera mounted above the workspace for top-down view

## Success / Fail or Pass Criteria

| Criteria | Pass Condition |
|----------|----------------|
| Object Detection | Correctly identifies color and shape of target objects with ≥90% accuracy |
| Pick Operation | Vacuum gripper successfully lifts target object |
| Place Operation | Object placed within ±10 mm of target location |
| Manual Control | All joints respond correctly to keyboard/gamepad input |
| Auto Mode Cycle | Complete pick-and-place cycle without human intervention |
| Safety Interlock | Motors and pump refuse to actuate when SAFETY_PIN is LOW; resume within 100 ms when SAFETY_PIN returns HIGH |
| Watchdog Recovery | Simulated firmware hang (infinite loop injection) triggers MCU reset within 2 seconds; system returns to known-safe state |
| Checksum Rejection | Corrupted serial frame (wrong `*XX` suffix) is rejected with `ERR: checksum mismatch`; valid frames with correct checksum are accepted |
| PID Wrist Levelling | Wrist servo maintains level within ±3° across ±45° arm pitch range; settles within 0.5 s after a 20° step disturbance |
| Fuzzy Motor Control | Motor speed adapts proportionally to pixel error: ≤80 PWM when error < 30 px, ≥200 PWM when error > 180 px |
| Digital Twin | Gazebo simulation mirrors real arm joint states in real time |
