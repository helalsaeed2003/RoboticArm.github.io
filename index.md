# PickMasters – 4-DOF Pick-and-Place Robotic Arm

**MEC 483: Mechatronics System Design | Abu Dhabi University – Spring 2026**

---

## Team Members

| Name | Role |
|------|------|
| Helal | CAD Design & System Integration |
| Leen | Computer Vision & Sensing |
| Mohamed | Hardware & Firmware Development |

---

## Problem Statement

In many industrial and educational settings, repetitive pick-and-place tasks are performed manually, leading to inefficiency, fatigue, and inconsistency. There is a need for an affordable, compact robotic arm system capable of autonomously identifying, picking, and placing objects using computer vision — while also supporting manual operator control for flexibility.

---

## Abstract

This project presents the design and development of a 4-DOF pick-and-place robotic arm controlled via a PC–Arduino architecture. The system operates in two modes: an **automatic mode** using OpenCV-based color and shape detection to identify and locate target objects, and a **manual mode** using keyboard or gamepad input via pygame. The arm features MG996R servo actuators for the shoulder, elbow, and wrist joints, a JGA25-370 DC gearmotor with quadrature encoder for 360° base rotation, and a custom vacuum suction end effector with IMU-based auto-levelling. A Python application on the PC handles computer vision, inverse kinematics, and user interface, while the Arduino R4 manages real-time servo control and PID-based motor control via USB serial communication. The project also includes a ROS2-integrated Gazebo digital twin for simulation and verification.

---

## [Background – Literature Review](Pages/literature-review.md)

---

## Methods

### Engineering Analysis

- [Sensing and Signal Architecture](Pages/sensing.md)
- [Actuation and Mechanical Drive](Pages/actuation.md)
- [Control Logic and Programming](Pages/control.md)
- [Advanced Modeling and Optimization](Pages/modeling.md)

### Testing

- [Prototype Construction](Pages/prototype.md)
- [Final Integration and Demonstration](Pages/integration.md)

---

## [Results](Pages/results.md)

---

## [Discussion](Pages/discussion.md)

---

## [Project Management – Gantt Chart](Pages/gantt.md)

---

## [Bibliography](Pages/bibliography.md)

---

## [Appendix](Pages/appendix.md)

---

## [Resources – Software & Hardware](Pages/resources.md)

---

> **Live Repository:** [github.com/PickMasters](https://github.com/PickMasters)
