# Actuation and Mechanical Drive

[← Back to Home](../index.md)

---

## Fluid Power and Kinematic Chains

The end effector uses a vacuum suction cup powered by a pneumatic pump/compressor (pump selection in progress — candidates include mini compressor, diaphragm pump, and venturi generator). The vacuum gripper provides a reliable, non-destructive gripping method suitable for flat-surfaced objects.

**Kinematic chain:** Base (360° rotation) → Shoulder (pitch) → Elbow (pitch) → Wrist (pitch) → End Effector (vacuum suction with auto-level). Total reach: ~400 mm.

## Electrical Actuation and Motor Selection

| Joint | Actuator | Specs | Notes |
|-------|----------|-------|-------|
| Base | JGA25-370 DC Gearmotor | 12V, with quadrature encoder | Full 360° rotation; PID-controlled |
| Shoulder | MG996R Servo | Stall torque: 11 kg·cm @ 6V | External 5–6V power required |
| Elbow | MG996R Servo | Stall torque: 11 kg·cm @ 6V | External 5–6V power required |
| Wrist | MG996R Servo | Stall torque: 11 kg·cm @ 6V | External 5–6V power required |

> **Note:** MG996R servos cannot be powered from the Arduino's 5V pin. An external 5–6V regulated supply is used, connected via the motor shield terminal.

## Embedded Controller Selection

- **Arduino R4:** Handles real-time PWM servo control, PID loop for the DC gearmotor, IMU reading, and serial communication with the PC.
- **L293D Motor Shield:** Used for servo header connections (pins 9 and 10 bypass the L293D chip) and DC motor driving during prototyping.
- **Planned motor driver:** L298N or TB6612FNG for final base motor control.

