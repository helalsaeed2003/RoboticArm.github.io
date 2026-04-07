# Control Logic and Programming

[← Back to Home](../index.md)

---

## Firmware Development

The Arduino R4 firmware:
- Receives serial commands from the PC in `<servo> <angle>` format
- Drives MG996R servos via PWM
- Runs a PID control loop for the JGA25-370 base motor using encoder feedback
- Reads IMU data for end effector auto-levelling
- Responds to `status` queries with current joint positions
- Initializes all servos to 90° on startup

## I/O Interfacing and Industrial Control

| Interface | Protocol | Description |
|-----------|----------|-------------|
| PC ↔ Arduino | USB Serial (pyserial) | Command/response for joint control |
| Arduino ↔ Servos | PWM | Direct servo position control |
| Arduino ↔ DC Motor | H-Bridge (L298N/TB6612FNG) | Speed and direction via PID |
| Arduino ↔ Encoder | Digital GPIO (interrupt) | Quadrature position feedback |
| Arduino ↔ IMU | I2C | Orientation data for auto-levelling |
| PC ↔ Camera | USB | Video feed for OpenCV processing |

## Communication and Networking

- **Primary link:** USB serial between PC and Arduino R4 (115200 baud)
- **ROS2 integration (planned):** ROS2 nodes on the PC will publish joint commands and subscribe to sensor feedback, enabling integration with the Gazebo digital twin for simulation and verification.

