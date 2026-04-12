# Sensing and Signal Architecture

[← Back to Home](../index.md)

---

## System Definition and Primary Sensing

The system uses a USB camera mounted above the workspace to capture a top-down view of the operating area. OpenCV processes the camera feed for real-time color and shape detection to identify target objects and determine their position coordinates.

An IMU (Inertial Measurement Unit) is mounted on the end effector to provide orientation feedback for the auto-levelling subsystem, ensuring the suction cup remains perpendicular to the target surface during operation.

## Signal Conditioning and Conversion

- **Camera feed:** Captured via USB; processed in Python using `opencv-python` for color space conversion (BGR → HSV), thresholding, contour detection, and shape classification.
- **IMU data:** Read via I2C on the Arduino R4; filtered and used in a feedback loop to drive wrist servo corrections for auto-levelling.
- **Encoder signals:** Quadrature encoder on the JGA25-370 base motor provides position feedback; decoded on the Arduino for PID control.

## Digital Logic and Data Presentation

- **User interface:** pygame-based display showing camera feed with detection overlays, system status, and manual control input.
- **Serial protocol:** PC ↔ Arduino communication via USB serial using `pyserial`. Command format: `<servo_number> <angle>` (e.g., `2 45`). A `status` command returns current joint positions.
