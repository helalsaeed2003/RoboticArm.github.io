# 4. System Overview

[← Back to Home](../index.md)

---

## 4.1 Functional Description

The system is a 4-DOF robotic arm (rotating base, shoulder, elbow, wrist) mounted on a two-wheeled mobile base. A dual end effector — vacuum suction cup plus motorised claw — is mounted on the wrist. An MPU6050 IMU at the wrist provides orientation feedback to keep the suction cup pointing downward. A camera attached to the arm base looks outward over the workspace; because it is mounted to the mobile base, the camera view shifts as the base pivots and drives, which the fuzzy-logic vision controller exploits to centre on a target.

High-level software runs on a host computer: the vision pipeline, manual control interface, and ROS digital twin. The host connects to the Arduino via USB serial. Low-level control runs on the Arduino: PWM to the servos, direction/speed control to the DC base motors via an L298N driver, on/off control of the vacuum pump via a relay, and continuous I²C reads from the IMU.

Power is delivered via two independent rails. The servo rail: four 3.7 V Li-ion cells in series → buck converter to 6 V → two 2200 µF capacitors across the rail. The pump rail: three 3.7 V Li-ion cells in series → relay-switched. The Arduino is USB-powered from the host.

![Figure 4: System block diagram showing all subsystems and their interconnections.](../media/figure4.png)

## 4.2 Subsystem Summary

Ten subsystems: mechanical arm (4× MG996R + PLA links); mobile base (2× DC motors + L298N + castor); end effector (12 V vacuum suction cup + motorised claw); power system (buck converter, capacitor bank, relay, 7× Li-ion cells across two rails); camera vision input (Logitech webcam); IMU (MPU6050); microcontroller (Arduino Uno R3); host computer; vision pipeline (Roboflow + Google Colab + Ultralytics YOLOv11); manual teleoperation (Bluetooth gamepad + Processing IDE sketch); digital twin (ROS 2 on Ubuntu VM).

---

[← Back to Home](../index.md)
