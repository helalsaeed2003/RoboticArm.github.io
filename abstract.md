# Abstract

[← Back to Home](index.md)

---

This project presents the design and development of a 4-DOF pick-and-place robotic arm controlled via a PC–Arduino architecture. The system operates in two modes: an **automatic mode** using OpenCV-based color and shape detection to identify and locate target objects, and a **manual mode** using keyboard or gamepad input via pygame. The arm features MG996R servo actuators for the shoulder, elbow, and wrist joints, a JGA25-370 DC gearmotor with quadrature encoder for 360° base rotation, and a custom vacuum suction end effector with IMU-based auto-levelling. A Python application on the PC handles computer vision, inverse kinematics, and user interface, while the Arduino R4 manages real-time servo control and PID-based motor control via USB serial communication. The project also includes a ROS2-integrated Gazebo digital twin for simulation and verification.

