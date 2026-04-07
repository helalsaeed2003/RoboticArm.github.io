# Advanced Modeling and Optimization

[← Back to Home](../index.md)

---

## Mathematical System Modeling

- **Forward/Inverse Kinematics:** Calculated using DH parameters for the 4-DOF arm; implemented in Python using `numpy`.
- **Workspace analysis:** ~400 mm reach envelope modeled in CAD (Autodesk Inventor).

### CAD Model

![CAD Model](Media/CAD.png)

*(Detailed kinematic equations and DH parameter table to be added.)*

## Stability and Feedback Control

- **PID control:** Applied to the JGA25-370 base motor for accurate angular positioning using encoder feedback.
- **Auto-levelling loop:** IMU feedback drives wrist servo corrections to maintain end effector orientation.

*(PID tuning results and stability analysis to be added.)*

## Intelligent Control and Final Design

- **Auto mode:** OpenCV detects objects by color and shape (no ML/AI training required). Object coordinates are transformed into joint angles via inverse kinematics and sent to the Arduino.
- **Manual mode:** Keyboard or gamepad input via pygame maps to joint-level commands sent over serial.

