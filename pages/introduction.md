# Introduction

[← Back to Home](../index.md)

---

## Team

| Member | Role |
|--------|------|
| Leen | Group Leader — Computer Vision & Project Coordination |
| Mohamed | End Effector, Suction System & ROS Digital Twin |
| Helal | Mechanical Design, Firmware, Power System & Manual Control |

---

## Executive Summary

This page describes the design, construction and validation of a smart pick-and-place robotic system designed for the semester project of MEC 483 Mechatronics System Design at Abu Dhabi University. It is a mobile 4-DOF robotic arm system with computer vision capabilities that can locate objects by scanning an environment using a camera, maneuver the end effector to them on a wheeled base and lift them with a dual end effector composed of a vacuum suction cup and a motorized claw.

The project development was entirely developed by three students from the three disciplines Mechanical, Electronic, Control and Computer. The hardware components used were either three-dimensional printed or laser cut or were purchased as standard electronic modules. The software has been developed in three different environments: Python on the host computer for computer vision and for high level control, Arduino C for embedded firmware and Processing IDE for the manual control interface. It features two modes: manual with a Bluetooth gamepad, and automatic with a computer vision model, YOLOv11.

The final system successfully shows the operation of all subsystems by hand, real-time detection of the objects with over 90% confidence in the majority of objects, automatic movement of the arm towards the objects, and automatic levelling of the wrist using inertial-measurement. The system was not fully autonomous pick-and-place due to the fact that the coordinate mapping between the camera frame and the arm workspace was not completed within the project window. The ROS digital twin was also incomplete since the joint transformation matrices were not set up properly at the end of the project.

![Figure 1: Final assembled robotic arm and mobile base.](../media/figure1.jpg)

---

## Problem Definition

### Problem Statement

Industrial pick-and-place tasks — sorting parts on an assembly line, packaging products, removing defective items from a stream — are repetitive, error-prone when performed by humans, and a natural fit for mechatronic automation. The same issues exist at smaller scale in laboratory environments, retail fulfilment, and any application requiring identification, localisation, and transportation of an object without human involvement. The difficulty is integrating mechanical structure, electrical power and signalling, control logic, and computer vision into one coherent platform.

The aim of this project is to construct a small-scale, educational pick-and-place robotic system that illustrates the essential concepts of mechatronic integration. The system should detect objects, move to them, pick them up, and also be controllable by a human operator. It should run on standard hobby-grade parts within budget and be repeatable — documented well enough for another team to reproduce it from the report.

### Impact Statement

The educational value lies in the integration challenge, not in any single subsystem. Commercial pick-and-place arms exist. Suction-cup grippers exist. Computer vision libraries exist. What is not available as a ready-made product is the integrated package developed by students and put together into a working system across each engineering discipline. The project results in a working prototype usable for teaching future students about sensor selection, power distribution, embedded control, and computer vision.

### Design Criteria and Constraints

**Criteria:** horizontal reach of 300–500 mm; 4-DOF arm structure; object detection accuracy ≥ 90%; manual control response time under 200 ms; both manual and automatic operating modes.

**Constraints:** Original design (no kits or pre-made assemblies); fabrication by 3D printing and laser cutting; microcontroller restricted to Arduino Uno R3 (ATmega328P); battery powered for mobile operation; target objects were a compass, a ball, and a bolt (with an egg and screw added mid-test by the instructor to test robustness); allowed languages were C, C++, and Python.

![Figure 2: The three target objects: bolt, ball, and compass.](../media/figure2.jpg)

---

## Project Management

### Team Composition and Responsibilities

**Leen — Group Leader.** Responsible for the computer vision pipeline and overall project coordination. Created the object detection pipeline: collecting data with the Logitech camera, labelling/augmenting it in Roboflow, training the YOLOv11 model in Google Colab, and building the Python detection script that integrates the trained model into the arm control system. Maintained weekly progress reports.

**Mohamed — Member.** In charge of the end effector and suction subsystem, plus ROS digital twin work. Selected and tested the vacuum pump, designed and printed the gripper portion of the dual end effector, and assembled the suction line. On the simulation side, set up Ubuntu in a VM, installed ROS 2, and imported the CAD model into the simulation environment. Joint transformation configuration was not finalised by the end of the project.

**Helal — Member.** Designed the mechanical structure, firmware, power system, and manual control code. Created the CAD model in Autodesk Inventor; printed all 3D parts; designed and wired the power distribution (buck converter rail and decoupling capacitor bank); wrote the unified Arduino firmware for servos, DC motors, pump relay, and IMU feedback loop; created the Processing gamepad interface for manual control.

### Project Timeline

The project ran over 13 weeks: weeks 1–3 sensing and signal architecture; weeks 4–6 actuation and mechanical drive; weeks 7–9 control logic and programming; weeks 10–12 modelling, integration and demonstration; week 13 final assembly and report writing.

![Figure 3: Project Gantt chart showing the thirteen-week schedule.](../media/figure3.png)

---

[← Back to Home](../index.md)
