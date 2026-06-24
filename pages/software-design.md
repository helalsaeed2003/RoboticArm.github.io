# 10. Software Design

[← Back to Home](../index.md)

---

## 10.1 Software Architecture Overview

Six layers, top to bottom: external model training services → persistent model storage and config files → host computer software → USB serial link → Arduino firmware → physical hardware. Manual mode (left half of Figure 14) uses Processing's DriveControl.pde → ArmController.ino. Automatic mode (right half) uses Python's detect_and_move.py → PickAndMove.ino. Both share the same USB serial link and the same physical hardware but run different control laws — PID on the wrist (manual) and Mamdani fuzzy on the base (auto). Standard flowchart symbology throughout: rounded terminators, hexagons (init), rectangles (process), double-bar rectangles (named function call), diamonds (decision), parallelograms (I/O), cylinders (storage), clouds (external services). Each firmware sketch's main loop refreshes the watchdog, parses serial, evaluates the safety interlock, and emits outputs.

![Figure 14: Software architecture and control-flow diagram. Six tiers from external training services down to physical hardware, with manual mode (left) and automatic mode (right) shown as parallel flowcharts using standard flowchart symbols.](../media/figure14.svg)

## 10.2 Computer Vision Pipeline

### 10.2.1 Dataset Collection

237 images per class collected with the Logitech camera across five classes: Ball, Bolt, Compass, Egg, Screw. Images shot individually and in groups, under varied lighting and angles, to expose the model to deployment-time variation.

![Figure 15: Sample of training images showing the five target object classes.](../media/figure15.png)

### 10.2.2 Annotation with Roboflow

Images uploaded to Roboflow; bounding boxes drawn by hand around every instance of each target object. Pre-processing: auto-orientation, resize, auto-contrast. Augmentation: random horizontal/vertical flip, small random rotation, random brightness variation — expands the dataset without further manual labelling.

![Figure 16: Roboflow annotation workspace showing bounding boxes drawn around objects.](../media/figure16.png)

### 10.2.3 Training with YOLOv11 on Google Colab

Trained on a Colab Tesla T4 GPU (~100× the matrix throughput of a typical laptop CPU). Dataset pulled via the Roboflow Python API. YOLOv11n trained for 50 epochs at 640×640. The "nano" variant chosen so inference is real-time on CPU at deployment. Ultralytics saves two checkpoints: `last.pt` (final epoch) and `best.pt` (highest validation mAP). `best.pt` (~5 MB) is the deployment file — copied into the inference script directory and renamed to `model_v2.pt`. Two training iterations: v1 (original images), v2 (expanded with egg and screw challenge images). Inference runs at ~15 fps on a normal laptop CPU at 640×480 — sufficient for the fuzzy loop in §9.7.

![Figure 17: Google Colab notebook showing the training cells and the dataset download.](../media/figure17.png)

### 10.2.4 Inference Script

`detect_and_move.py` loads `model_v2.pt`, opens the USB camera via OpenCV's VideoCapture (DirectShow backend), and opens an Arduino serial port at 9600 baud. Each loop iteration: run model in tracking mode (persistent IDs across frames), overlay bounding boxes + reference lines + the target box in the centre.

**Lock-and-track behaviour:** when no lock is held, pick the detection with the highest confidence above 50% and lock onto its tracking ID. Once locked, ignore other detections until the operator presses `n` to release.

**Per-frame errors:** signed horizontal and vertical pixel offset between the locked object's centre and the target-box centre, compared against a 30-pixel dead zone. Horizontal error out of band → `FUZZY_PIVOT <error>`. Horizontal in band, vertical out → `FUZZY_DRIVE <error>`. Both in band and object bounding box inside the target box → `STOP` and centring complete. A frame-level cooldown keeps the serial link unsaturated. Pulse-based motion (host re-sends pulses as needed) mirrors the dead-man's-switch design in §9.7.

![Figure 18: Live detection screenshot showing YOLOv11 identifying all five object classes with confidence scores.](../media/figure18.jpg)

## 10.3 Manual Control Software

Processing IDE — chosen because GameControlPlus already handles Bluetooth gamepads and Processing's `draw()` loop fits gamepad polling cleanly. Polls every axis at 50 Hz, applies a deadzone, and converts to serial commands. Mapping: left stick — base wheels (up/down forward/back, left/right rotate); right stick — hand/claw override; D-pad — shoulder and elbow up/down; face buttons — pump on/off, IMU calibration, end-effector control override.

## 10.4 Robot Operating System 2 Simulation

ROS 2 on an Ubuntu VM. Goal: a digital twin showing live joint positions and allowing trajectory testing virtually before running on hardware. The CAD model was successfully imported and rendered in RViz. Joint transformations were not finalised by the project deadline, so the model renders but does not articulate correctly.

![Figure 19: RViz screenshot showing the imported arm model in the simulation environment.](../media/figure19.jpg)

---

[← Back to Home](../index.md)
