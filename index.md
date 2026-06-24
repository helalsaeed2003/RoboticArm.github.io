# PickMasters — Smart Pick-and-Place Robotic System

**MEC 483: Mechatronics System Design | Abu Dhabi University — Spring 2026**

| Member | Role |
|--------|------|
| Leen | Group Leader — Computer Vision & Project Coordination |
| Mohamed | End Effector, Suction System & ROS Digital Twin |
| Helal | Mechanical Design, Firmware, Power System & Manual Control |

![Final assembled robotic arm and mobile base](media/figure1.jpg)

---

## Executive Summary

- 4-DOF robotic arm on a two-wheeled mobile base with computer vision
- Dual end effector: vacuum suction cup + motorised claw
- Two operating modes: manual (Bluetooth gamepad) and automatic (YOLOv11 object detection)
- Real-time detection at >90% confidence; IMU-based wrist auto-levelling

[Read more →](pages/executive-summary.md)

---

## Problem Definition

- Industrial pick-and-place tasks are repetitive and error-prone when done manually
- Goal: build an educational-scale integrated mechatronic system from hobby-grade parts
- Design criteria: 300–500 mm reach, 4-DOF, ≥90% detection accuracy, <200 ms manual response

[Read more →](pages/problem-definition.md)

---

## Project Management

- Three-member team spanning mechanical, electronic, and software disciplines
- 13-week timeline from sensing architecture through final integration
- Weekly progress tracking with clear per-member deliverables

[Read more →](pages/project-management.md)

---

## System Design

- **System Overview** — Block diagram of all subsystems and their interconnections
  [Read more →](pages/system-overview.md)
- **Mechanical Design** — 4-DOF kinematic chain, 3D-printed PLA links, laser-cut acrylic base
  [Read more →](pages/mechanical-design.md)
- **Electrical Design** — Two-rail power architecture, Arduino Uno R3, full pin allocation
  [Read more →](pages/electrical-design.md)

---

## Sensing & Actuation

- **Sensing and Signal Architecture** — USB camera, MPU6050 IMU, Bluetooth gamepad
  [Read more →](pages/sensing.md)
- **Actuation** — MG996R servos, DC gear motors, 12 V vacuum pump, relay switching
  [Read more →](pages/actuation.md)

---

## Software & Control

- **Embedded Control Logic** — PID wrist levelling, Mamdani fuzzy vision-guided motion, safety interlock
  [Read more →](pages/embedded-control.md)
- **Software Design** — YOLOv11 vision pipeline, Processing manual control, ROS 2 digital twin
  [Read more →](pages/software-design.md)

---

## Analysis & Validation

- **Mathematical Modelling** — DH parameters, static torque verification, Bode analysis of the wrist PID loop
  [Read more →](pages/modelling.md)
- **Integration and Testing** — Assembly sequence, calibration, test results and acknowledged limitations
  [Read more →](pages/integration-testing.md)

---

## Reflections

- **Lessons Learned** — Component ratings, decoupling capacitors, power rail separation, realistic training data
  [Read more →](pages/lessons-learned.md)
- **Future Work** — Complete auto pick-and-place, functional ROS 2 twin, closed-loop DC motor control
  [Read more →](pages/future-work.md)

---

## Appendices

- [References](pages/references.md)
- [Bill of Materials](pages/bill-of-materials.md)
- [User Manual](pages/user-manual.md)
- [Software Listings](pages/software-listings.md)

---

> **Live Repository:** [github.com/PickMasters](https://github.com/PickMasters)
