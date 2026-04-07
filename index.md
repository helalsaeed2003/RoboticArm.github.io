# PickMasters — 4-DOF Pick-and-Place Robotic Arm

**MEC 483: Mechatronics System Design**  
**Abu Dhabi University — Spring 2026**

---

## Team Members

| Name | Role |
|------|------|
| Helal | CAD Design & System Integration |
| Leen | Computer Vision & Sensing |
| Mohamed | Hardware & Firmware Development |

---

## Problem Statement

In many industrial and educational settings, repetitive pick-and-place tasks are performed manually, leading to inefficiency, inconsistency, and operator fatigue. There is a need for an affordable, compact robotic arm capable of autonomously identifying, picking, and placing objects based on color and shape — while also supporting manual control for flexible operation.

---

## Abstract

This project presents the design and development of a 4-degree-of-freedom (4-DOF) pick-and-place robotic arm for the MEC 483 Mechatronics System Design course. The system integrates mechanical design (Autodesk Inventor CAD, 3D-printed structure), electrical actuation (MG996R servos, JGA25-370 DC gearmotor with encoder), embedded control (Arduino R4), and intelligent perception (OpenCV-based color/shape detection) under a PC-Arduino serial communication architecture. The arm features a custom vacuum suction end effector with IMU-based auto-levelling and supports both autonomous and manual (keyboard/gamepad) operation modes. A ROS2-based digital twin in Gazebo accompanies the physical prototype for simulation and validation.

---

## Background — Literature Review

Pick-and-place robotic arms are widely used in manufacturing, logistics, and laboratory automation. Commercial solutions such as the Dobot Magician and uArm Swift Pro demonstrate the viability of small-scale desktop arms for educational and light-duty applications. Research in affordable robotics has shown that 3D-printed structures combined with hobby-grade servos can achieve functional performance at a fraction of the cost of commercial units.

OpenCV-based object detection using color segmentation (HSV thresholding) and shape detection (contour analysis) provides a robust, training-free approach suitable for controlled environments with known object sets. This eliminates the need for computationally expensive deep learning models such as YOLOv4, which was considered and ruled out for this project.

Vacuum suction grippers offer advantages over mechanical claws for flat and smooth objects, providing consistent grip force without complex finger mechanisms. IMU-based levelling ensures the suction cup maintains proper orientation regardless of arm pose.

*References are listed in the [Bibliography](#bibliography) section.*

---

## Methods

### Engineering Analysis (Calculations and Simulations)

#### Sensing and Signal Architecture

##### System Definition and Primary Sensing

The primary sensing system consists of a USB camera mounted above the workspace, feeding frames to a PC running OpenCV. The camera captures the workspace in real time, and the vision pipeline identifies target objects by color (HSV thresholding) and shape (contour analysis with polygon approximation). Object coordinates in pixel space are transformed to workspace coordinates using a calibrated homography matrix.

An MPU6050 IMU is mounted on the end effector to provide real-time orientation feedback for the auto-levelling subsystem.

##### Signal Conditioning and Conversion

- **Camera → PC:** USB digital signal; no analog conditioning required
- **IMU → Arduino:** I²C digital communication; onboard DMP handles sensor fusion
- **Encoder → Arduino:** Quadrature digital pulses from the JGA25-370 encoder; decoded in firmware for position feedback

##### Digital Logic and Data Presentation

The PC serves as the central intelligence hub, running Python scripts that:
- Process camera frames and display detected objects via OpenCV GUI windows
- Compute inverse kinematics (IK) to convert target coordinates to joint angles
- Provide a pygame-based interface showing arm status, detected objects, and control mode

---

#### Actuation and Mechanical Drive

##### Fluid Power and Kinematic Chains

The end effector uses a vacuum suction system to grip objects. Pump/compressor selection is under evaluation with the following candidates:
- Mini air compressor
- Diaphragm pump
- Venturi generator

The kinematic chain consists of 4 DOF:
1. **Base** — 360° rotation (DC gearmotor)
2. **Shoulder** — angular positioning (MG996R servo)
3. **Elbow** — angular positioning (MG996R servo)
4. **Wrist** — angular positioning (MG996R servo)

The arm has an approximate reach of **400 mm**, designed and modeled in Autodesk Inventor.

##### Electrical Actuation and Motor Selection

| Joint | Actuator | Specification | Driver |
|-------|----------|---------------|--------|
| Base | JGA25-370 DC Gearmotor | 12V, quadrature encoder, 360° rotation | L298N / TB6612FNG |
| Shoulder | MG996R Servo | 10 kg·cm torque, 4.8–6V | Direct PWM from Arduino |
| Elbow | MG996R Servo | 10 kg·cm torque, 4.8–6V | Direct PWM from Arduino |
| Wrist | MG996R Servo | 10 kg·cm torque, 4.8–6V | Direct PWM from Arduino |

> **Note:** MG996R servos require external 5–6V power supply; they cannot draw from the Arduino's 5V pin.

##### Embedded Controller Selection

**Arduino R4** was selected as the real-time embedded controller. It handles:
- PWM generation for servo control
- PID loop for DC gearmotor position control via encoder feedback
- Serial communication with the PC (USB)
- IMU data reading for end effector levelling

A Raspberry Pi was considered and eliminated — the PC handles all intelligent processing (vision, IK, UI), keeping the embedded side focused on real-time actuation.

---

#### Control Logic and Programming

##### Firmware Development

The Arduino firmware accepts serial commands from the PC in the format:

```
<servo_number> <angle>
```

For example, `2 45` moves servo 2 to 45°. A `status` command returns current positions of all servos. All servos initialize to 90° on startup.

Current implementation (Week 4): 2-DOF serial servo controller with base on pin 9 and shoulder on pin 10 via L293D motor shield headers.

##### I/O Interfacing and Industrial Control

- **PC ↔ Arduino:** USB serial via `pyserial` (Python) at 9600 baud
- **Arduino → Servos:** PWM signals on dedicated pins (pins 9, 10 on L293D shield bypass the motor driver chip)
- **Arduino → DC Motor:** H-bridge driver (L298N or TB6612FNG) with encoder feedback
- **Arduino ← IMU:** I²C bus (SDA/SCL)

##### Communication and Networking

The system uses a straightforward **USB serial** communication link between the PC and Arduino. The PC sends target joint angles computed from IK; the Arduino executes them. No wireless or networked communication is currently planned for the physical system.

For the digital twin, **ROS2** topics will handle communication between nodes (vision, control, simulation).

---

#### Advanced Modeling and Optimization

##### Mathematical System Modeling

- **Forward Kinematics:** DH parameter-based model for the 4-DOF arm
- **Inverse Kinematics:** Geometric/analytical IK solver to convert (x, y, z) workspace coordinates to joint angles (θ₁, θ₂, θ₃, θ₄)
- **DC Motor Model:** Transfer function modeling for the base joint gearmotor including encoder feedback

##### Stability and Feedback Control

- **Base Joint PID:** Closed-loop position control using quadrature encoder feedback; PID tuning to minimize overshoot and settling time
- **End Effector Levelling:** IMU-based feedback loop to maintain suction cup perpendicularity to the work surface

##### Intelligent Control and Final Design

- **Auto Mode:** OpenCV detects objects → IK computes joint angles → serial commands sent → arm executes pick-and-place sequence
- **Manual Mode:** Keyboard or gamepad input via pygame → joint-level or Cartesian control → serial commands sent to Arduino

---

### Testing

#### Prototype Construction

##### Initial Design

- CAD model designed in Autodesk Inventor (~400 mm reach)
- Parts being scaled and prepared for 3D printing
- Motor mounting holes and wiring channels added to CAD

##### Final Design

*To be updated as the project progresses.*

#### Final Integration and Demonstration

##### Testing Environment

The arm will be tested in a controlled lab environment with:
- Flat workspace surface
- Known set of colored geometric objects (cubes, cylinders)
- Consistent overhead lighting for reliable vision detection
- USB camera mounted at a fixed position above the workspace

##### Success / Fail or Pass Criteria

| Test | Pass Criteria |
|------|---------------|
| Object Detection | Correctly identifies color and shape of ≥ 90% of test objects |
| Pick Accuracy | Suction cup contacts target object within ±5 mm of center |
| Place Accuracy | Object placed within ±10 mm of target location |
| Full Cycle Time | Complete pick-and-place cycle in < 15 seconds |
| Manual Control | All joints respond correctly to keyboard/gamepad input |
| Auto-Levelling | End effector maintains horizontal orientation within ±5° |
| Digital Twin | Gazebo model mirrors physical arm movements in real time |

---

## Results

*To be updated with experimental data and test outcomes.*

---

## Discussion

*To be updated with analysis of results, challenges encountered, and lessons learned.*

---

## Project Management Summary

### Gantt Chart

*[Link to updated Gantt Chart (Excel/MS Project)](#)*

| Week | Milestone |
|------|-----------|
| 1–2 | Project scoping, team formation, initial research |
| 3 | System architecture defined, component selection, CAD started |
| 4 | CAD refinement & 3D printing, hardware procurement, initial firmware |
| 5–6 | Assembly, vision pipeline, expanded firmware |
| 7–8 | Integration, IK implementation, auto mode |
| 9 | ROS2 & Gazebo digital twin |
| 10–11 | Testing, debugging, optimization |
| 12 | Final demo & documentation |

---

## Bibliography

1. Corke, P. (2017). *Robotics, Vision and Control*. Springer.
2. OpenCV Documentation. [https://docs.opencv.org/](https://docs.opencv.org/)
3. Arduino Reference. [https://www.arduino.cc/reference/en/](https://www.arduino.cc/reference/en/)
4. ROS2 Documentation. [https://docs.ros.org/](https://docs.ros.org/)
5. MG996R Servo Datasheet. TowerPro.
6. JGA25-370 DC Gearmotor Datasheet.

*Additional references to be added as the project develops.*

---

## Appendix

### A. Arduino Serial Protocol

| Command | Description | Example |
|---------|-------------|---------|
| `<servo> <angle>` | Move servo to angle | `2 45` |
| `status` | Print current positions | — |

### B. Software Stack

| Component | Tool / Library |
|-----------|---------------|
| CAD | Autodesk Inventor |
| PC Vision | OpenCV (`opencv-python`) |
| PC Control & UI | `pygame`, `numpy` |
| Serial Communication | `pyserial` |
| Embedded Firmware | Arduino IDE (C++) |
| Simulation | ROS2, Gazebo |

### C. Bill of Materials

| Item | Qty | Notes |
|------|-----|-------|
| MG996R Servo | 3 | Shoulder, elbow, wrist |
| JGA25-370 DC Gearmotor | 1 | Base joint, with encoder |
| Arduino R4 | 1 | Main controller |
| L293D Motor Shield | 1 | Current test setup |
| L298N / TB6612FNG | 1 | Planned motor driver |
| MPU6050 IMU | 1 | End effector levelling |
| USB Camera | 1 | Workspace vision |
| Vacuum Pump (TBD) | 1 | End effector suction |
| 3D Printed Parts | — | PLA, Autodesk Inventor designs |
| Jumper Wires | — | Assorted |
| External 5–6V PSU | 1 | Servo power |

---

## Resources — Pre-Existing Software / Hardware

- **Software:** Python 3.x, Arduino IDE, Autodesk Inventor (student license), ROS2 Humble, Gazebo
- **Hardware:** Personal laptops, Abu Dhabi University makerspace (3D printers), lab workspace

---

> *Last updated: April 2026*  
> *PickMasters — MEC 483 Mechatronics System Design — Abu Dhabi University*
