# Executive Summary

[← Back to Home](../index.md)

---

The report describes the design, construction and validation of a smart pick and place robotic system designed for the semester project of MEC 483 Mechatronics System Design at Abu Dhabi University. It is a mobile 4-DOF robotic arm system with computer vision capabilities that can locate objects by scanning an environment using a camera, maneuver the end effector to them on a wheeled base and lift them with a dual end effector composed of a vacuum suction cup and a motorized claw.

The project development was entirely developed by three students from the three disciplines Mechanical, Electronic, Control and Computer. The hardware components used were either three-dimensional printed or laser cut or were purchased as standard electronic modules. The software has been developed in three different environments: Python on the host computer for computer vision and for high level control, Arduino C for embedded firmware and Processing IDE for the manual control interface. It features two modes: manual with a Bluetooth gamepad, and automatic with a computer vision model, YOLOv11.

The final system successfully shows the operation of all subsystems by hand, real-time detection of the objects with over 90% confidence in the majority of objects, automatic movement of the arm towards the objects, and automatic levelling of the wrist using inertial-measurement. The system was not fully autonomous pick-and-place due to the fact that the coordinate mapping between the camera frame and the arm workspace was not completed within the project window. The ROS digital twin was also incomplete since the joint transformation matrices were not set up properly at the end of the project.

![Figure 1: Final assembled robotic arm and mobile base.](../media/figure1.jpg)

---

[← Back to Home](../index.md)
