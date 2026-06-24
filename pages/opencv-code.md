# Vision Development Scripts

[← Back to Home](../index.md)

---

These are the computer vision scripts developed during the project. The production version is `detect_and_move.py` in [Final Code](final-code.md). This directory contains the development iterations and utility scripts used during the vision pipeline build-out.

---

## `detect_and_move.py`

> Python 3 | 258 lines

Development copy of the main vision-and-control script. Functionally equivalent to the final version — loads the YOLOv11n model, tracks objects with persistent IDs, and sends fuzzy-logic motion commands over serial. This copy was the working version during active development; the production copy in `Final Code/` is the clean release.

---

## `detect_v2.py`

> Python 3 | 108 lines

Detection-only script without motor control. Loads the YOLOv11n model, opens the camera, runs inference, and displays annotated frames with bounding boxes and confidence scores. Used to validate model accuracy and tune confidence thresholds before wiring up the serial control loop.

```python
import cv2
from ultralytics import YOLO
import serial
import time

model = YOLO("model_v2.pt")
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Error: Could not open camera")
    exit()

try:
    arduino = serial.Serial("COM10", 9600, timeout=1)
    time.sleep(2)
    print("Arduino connected!")
except:
    print("Arduino not found")
    arduino = None

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CENTER_X = FRAME_WIDTH // 2
CENTER_Y = FRAME_HEIGHT // 2
DEAD_ZONE = 30

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model.track(frame, persist=True, verbose=False)
    annotated = results[0].plot()

    cv2.line(annotated, (CENTER_X, 0), (CENTER_X, FRAME_HEIGHT), (255, 0, 0), 2)
    cv2.line(annotated, (0, CENTER_Y), (FRAME_WIDTH, CENTER_Y), (255, 0, 0), 2)

    boxes = results[0].boxes
    if boxes is not None and len(boxes) > 0:
        best_idx = boxes.conf.argmax()
        best_box = boxes[best_idx]
        x1, y1, x2, y2 = map(int, best_box.xyxy[0])
        obj_cx = (x1 + x2) // 2
        obj_cy = (y1 + y2) // 2

        class_id = int(best_box.cls[0])
        class_name = results[0].names[class_id]
        confidence = float(best_box.conf[0])

        cv2.circle(annotated, (obj_cx, obj_cy), 8, (0, 0, 255), -1)

        error_x = obj_cx - CENTER_X
        error_y = CENTER_Y - obj_cy

        command = None
        status = ""

        if abs(error_x) > DEAD_ZONE:
            if error_x > 0:
                command = f"PIVOT_RIGHT {min(abs(error_x), 200)}"
                status = f"PIVOT RIGHT (err={error_x})"
            else:
                command = f"PIVOT_LEFT {min(abs(error_x), 200)}"
                status = f"PIVOT LEFT (err={error_x})"
        elif abs(error_y) > DEAD_ZONE:
            if error_y > 0:
                command = f"DRIVE_FWD {min(abs(error_y), 200)}"
                status = f"DRIVE FWD (err={error_y})"
            else:
                command = f"DRIVE_BACK {min(abs(error_y), 200)}"
                status = f"DRIVE BACK (err={error_y})"
        else:
            command = "STOP"
            status = f"CENTERED on {class_name}"

        color = (0, 255, 0) if command == "STOP" else (0, 255, 255)
        cv2.putText(annotated, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(annotated, f"{class_name} ({confidence:.2f})",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        if command and arduino:
            try:
                arduino.write((command + "\n").encode())
                print(f"Sent: {command}")
            except:
                pass
    else:
        cv2.putText(annotated, "No object detected", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        if arduino:
            try:
                arduino.write(b"STOP\n")
            except:
                pass

    cv2.imshow("Pick & Move Detection", annotated)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if arduino:
    arduino.close()
```

---

## `capture.py`

> Python 3 | 16 lines

Simple webcam capture utility. Opens the camera, displays a live feed, and saves frames to disk when a key is pressed. Used to collect the training dataset for the YOLOv11 model.

```python
import cv2

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("Webcam", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

## `model_v2.pt`

> YOLOv11n checkpoint | ~5 MB

The trained model weights. Five classes: Ball, Bolt, Compass, Egg, Screw. Trained for 50 epochs at 640×640 on a Google Colab Tesla T4 GPU using data annotated in Roboflow. This is the same file as `best.pt` in `Final Code/` — renamed for the inference scripts.

---

[← Back to Home](../index.md)
