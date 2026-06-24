# Vision Development Scripts

[← Back to Home](../index.md)

---

These are the computer vision scripts developed during the project. The production version is `detect_and_move.py` in [Final Code](final-code.md). This directory contains the development iterations and utility scripts used during the vision pipeline build-out.

---

## [`detect_and_move.py`](../OpenCV/PICK%26PLACE/detect_and_move.py)

Development copy of the main vision-and-control script. Functionally equivalent to the final version — loads the YOLOv11n model, tracks objects with persistent IDs, and sends fuzzy-logic motion commands over serial. This copy was the working version during active development; the production copy in <code>Final Code/</code> is the clean release.

<details>
<summary>Python 3 | 258 lines</summary>
<div class="code-scroll">
<pre><code>&quot;&quot;&quot;
detect_and_move.py
==================

Vision + control loop for the PickMasters arm.

The camera feed is split by ONE vertical and ONE horizontal line.  Where the two
lines cross there is a target BOX.  The job of this program is to drive the robot
(DC motors only) until the chosen item sits completely inside that box:

  * VERTICAL line   -&gt; handled by SPINNING the base (pivot the wheels left/right)
                       so the item lines up on the vertical (X) axis.
  * HORIZONTAL line -&gt; handled by MOVING the base forward/back (drive both wheels)
                       so the item lines up on the horizontal (Y) axis.

Locking behaviour
-----------------
The program will only LOCK onto an item once it is seen with &gt;= 80% confidence.
After that it stays focused on the SAME item and keeps centering it, even if the
confidence drops, until you press the &quot;next&quot; key (n).  Then it releases the lock
and is free to pick the next &gt;= 80% item.

Keys
----
  n : release the current lock and look for the next item
  s : send an emergency STOP to the Arduino
  q : quit

Serial protocol (sent to the Arduino, one command per line)
-----------------------------------------------------------
  PIVOT_LEFT  &lt;speed&gt;   spin base left
  PIVOT_RIGHT &lt;speed&gt;   spin base right
  DRIVE_FWD   &lt;speed&gt;   move base forward
  DRIVE_BACK  &lt;speed&gt;   move base backward
  STOP                  stop all DC motors
&quot;&quot;&quot;

import os
import cv2
from ultralytics import YOLO
import serial
import time

# --------------------------------------------------------------------------- #
#  Configuration
# --------------------------------------------------------------------------- #
SERIAL_PORT = &quot;COM10&quot;          # change to match your Arduino port
BAUD_RATE   = 9600

# Locate model_v2.pt next to this script so the path works on any machine.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, &quot;model_v2.pt&quot;)

CAMERA_INDEX = 0

FRAME_WIDTH  = 640
FRAME_HEIGHT = 480
CENTER_X     = FRAME_WIDTH  // 2
# Shift the target (box + horizontal line) lower on the Y axis. Increase
# BOX_Y_OFFSET to push it further down the frame.
BOX_Y_OFFSET = 80
CENTER_Y     = FRAME_HEIGHT // 2 + BOX_Y_OFFSET

# Half-size of the target box drawn at the crossing of the two lines.
# The item must fit COMPLETELY inside this box.
BOX_HALF_W = 55
BOX_HALF_H = 55

BOX_LEFT   = CENTER_X - BOX_HALF_W
BOX_RIGHT  = CENTER_X + BOX_HALF_W
BOX_TOP    = CENTER_Y - BOX_HALF_H
BOX_BOTTOM = CENTER_Y + BOX_HALF_H

# How close (in pixels) the item centre must be to a line before we stop nudging
# on that axis.  Keep this smaller than the box so the item ends up well inside.
DEAD_ZONE_X = 30
DEAD_ZONE_Y = 30

LOCK_CONFIDENCE = 0.50         # must reach this to LOCK onto an item

PIVOT_SPEED = 90               # PWM (0-255) the Arduino uses while spinning base
DRIVE_SPEED = 130              # PWM (0-255) the Arduino uses while driving base

COOLDOWN_MAX = 6               # frames to wait between motion pulses (higher = gentler)


# --------------------------------------------------------------------------- #
#  Setup
# --------------------------------------------------------------------------- #
model = YOLO(MODEL_PATH)
# Use the DirectShow backend on Windows -- the default backend can hang for a
# long time while opening the camera.
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if not cap.isOpened():
    raise RuntimeError(
        f&quot;Could not open camera index {CAMERA_INDEX}. &quot;
        f&quot;Try a different CAMERA_INDEX (0, 1, 2 ...).&quot;
    )

try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(&quot;Arduino connected!&quot;)
except Exception as e:
    print(f&quot;Arduino not found: {e}&quot;)
    arduino = None


def send(command):
    &quot;&quot;&quot;Send a single line command to the Arduino (and echo to the console).&quot;&quot;&quot;
    print(f&quot;Sent: {command}&quot;)
    if arduino:
        try:
            arduino.write((command + &quot;\n&quot;).encode())
        except Exception as e:
            print(f&quot;Send error: {e}&quot;)


# --------------------------------------------------------------------------- #
#  Main loop
# --------------------------------------------------------------------------- #
locked_id = None               # track id of the item we are focused on
cooldown  = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Use tracking so each item keeps a stable id between frames.  This is what
    # lets us stay locked on one item even when its confidence drops.
    results = model.track(frame, persist=True, verbose=False)
    annotated = results[0].plot()

    # ---- Draw the two reference lines and the target box ------------------- #
    cv2.line(annotated, (CENTER_X, 0), (CENTER_X, FRAME_HEIGHT), (255, 0, 0), 2)
    cv2.line(annotated, (0, CENTER_Y), (FRAME_WIDTH, CENTER_Y), (255, 0, 0), 2)
    cv2.rectangle(annotated, (BOX_LEFT, BOX_TOP), (BOX_RIGHT, BOX_BOTTOM),
                  (0, 255, 255), 2)

    boxes = results[0].boxes

    # Build a lookup of the boxes that currently have a track id.
    tracked = {}
    if boxes is not None and boxes.id is not None:
        for i in range(len(boxes)):
            tid = int(boxes.id[i])
            tracked[tid] = boxes[i]

    # ---- Acquire a lock if we don&#x27;t have one ------------------------------ #
    if locked_id is None or locked_id not in tracked:
        # Look for the highest-confidence item that clears the 80% threshold.
        best_id, best_conf = None, 0.0
        for tid, box in tracked.items():
            conf = float(box.conf[0])
            if conf &gt;= LOCK_CONFIDENCE and conf &gt; best_conf:
                best_id, best_conf = tid, conf

        if best_id is not None:
            locked_id = best_id
            print(f&quot;LOCKED onto item id {locked_id} ({best_conf:.2f})&quot;)
        else:
            # Nothing to lock onto -&gt; make sure the robot is stopped.
            if cooldown == 0:
                send(&quot;STOP&quot;)
                cooldown = COOLDOWN_MAX
            locked_id = None

    # ---- Drive toward the locked item ------------------------------------- #
    if locked_id is not None and locked_id in tracked:
        box = tracked[locked_id]
        class_name = results[0].names[int(box.cls[0])]
        confidence = float(box.conf[0])

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        obj_cx = (x1 + x2) // 2
        obj_cy = (y1 + y2) // 2

        cv2.circle(annotated, (obj_cx, obj_cy), 8, (0, 0, 255), -1)

        # Is the whole item inside the box?
        fully_inside = (x1 &gt;= BOX_LEFT and x2 &lt;= BOX_RIGHT and
                        y1 &gt;= BOX_TOP  and y2 &lt;= BOX_BOTTOM)

        command = None
        if fully_inside:
            # Item is completely inside the box -&gt; we&#x27;re done. Stop and quit.
            send(&quot;STOP&quot;)
            status, color = f&quot;DONE - {class_name} in box&quot;, (0, 255, 0)
            cv2.putText(annotated, status, (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.imshow(&quot;Pick &amp; Move&quot;, annotated)
            cv2.waitKey(800)          # show the result briefly
            print(f&quot;Item id {locked_id} ({class_name}) fully inside box. Quitting.&quot;)
            break
        elif obj_cx &lt; CENTER_X - DEAD_ZONE_X:
            # Item is left of the vertical line -&gt; spin base left.
            command = f&quot;PIVOT_LEFT {PIVOT_SPEED}&quot;
            status, color = &quot;SPIN BASE LEFT&quot;, (0, 255, 255)
        elif obj_cx &gt; CENTER_X + DEAD_ZONE_X:
            command = f&quot;PIVOT_RIGHT {PIVOT_SPEED}&quot;
            status, color = &quot;SPIN BASE RIGHT&quot;, (0, 255, 255)
        elif obj_cy &lt; CENTER_Y - DEAD_ZONE_Y:
            # Item is above the horizontal line -&gt; move base forward.
            command = f&quot;DRIVE_FWD {DRIVE_SPEED}&quot;
            status, color = &quot;MOVE BASE FORWARD&quot;, (255, 165, 0)
        elif obj_cy &gt; CENTER_Y + DEAD_ZONE_Y:
            command = f&quot;DRIVE_BACK {DRIVE_SPEED}&quot;
            status, color = &quot;MOVE BASE BACK&quot;, (255, 165, 0)
        else:
            # Centre is on the cross but the box isn&#x27;t fully contained yet
            # (item bigger than the dead zone) -&gt; nudge it the rest of the way.
            command = None
            status, color = &quot;CENTERING...&quot;, (0, 255, 0)

        if cooldown == 0:
            send(command if command else &quot;STOP&quot;)
            cooldown = COOLDOWN_MAX

        cv2.putText(annotated, status, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(annotated, f&quot;LOCK id{locked_id} {class_name} {confidence:.2f}&quot;,
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    elif locked_id is not None:
        # We have a lock but lost sight of it this frame -&gt; hold position.
        if cooldown == 0:
            send(&quot;STOP&quot;)
            cooldown = COOLDOWN_MAX
        cv2.putText(annotated, f&quot;SEARCHING for locked id {locked_id}&quot;,
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    else:
        cv2.putText(annotated, &quot;No &gt;=80% item to lock onto&quot;,
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    if cooldown &gt; 0:
        cooldown -= 1

    cv2.imshow(&quot;Pick &amp; Move&quot;, annotated)
    key = cv2.waitKey(1) &amp; 0xFF
    if key == ord(&#x27;q&#x27;):
        break
    elif key == ord(&#x27;n&#x27;):
        # Release the lock so the next &gt;=80% item can be chosen.
        print(f&quot;Released lock on id {locked_id}&quot;)
        locked_id = None
        send(&quot;STOP&quot;)
    elif key == ord(&#x27;s&#x27;):
        send(&quot;STOP&quot;)

send(&quot;STOP&quot;)
cap.release()
cv2.destroyAllWindows()
if arduino:
    arduino.close()</code></pre>
</div>
</details>

---

## [`detect_v2.py`](../OpenCV/PICK%26PLACE/detect_v2.py)

Detection-only script without motor control. Loads the YOLOv11n model, opens the camera, runs inference, and displays annotated frames with bounding boxes and confidence scores. Used to validate model accuracy and tune confidence thresholds before wiring up the serial control loop.

<details>
<summary>Python 3 | 109 lines</summary>
<div class="code-scroll">
<pre><code>import cv2
from ultralytics import YOLO
import serial
import time

model = YOLO(r&quot;C:\Users\Helal\Desktop\School\term 13\Mechatronics\Github\RoboticArm.github.io\OpenCV\PICK&amp;PLACE\model_v2.pt&quot;)
cap = cv2.VideoCapture(0)

try:
    arduino = serial.Serial(&#x27;COM10&#x27;, 9600, timeout=1)
    time.sleep(2)
    print(&quot;Arduino connected!&quot;)
except Exception as e:
    print(f&quot;Arduino not found: {e}&quot;)
    arduino = None

FRAME_WIDTH    = 640
FRAME_HEIGHT   = 480
CENTER_X       = FRAME_WIDTH  // 2
CENTER_Y       = FRAME_HEIGHT // 2
DEAD_ZONE_X    = 50
DEAD_ZONE_Y    = 50
TURN_SPEED     = 15
SHOULDER_SPEED = 10
COOLDOWN_MAX   = 3

cooldown = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    annotated = results[0].plot()

    cv2.line(annotated, (CENTER_X, 0), (CENTER_X, FRAME_HEIGHT), (255, 0, 0), 2)
    cv2.line(annotated, (0, CENTER_Y), (FRAME_WIDTH, CENTER_Y), (255, 0, 0), 2)
    cv2.rectangle(annotated,
                  (CENTER_X - DEAD_ZONE_X, CENTER_Y - DEAD_ZONE_Y),
                  (CENTER_X + DEAD_ZONE_X, CENTER_Y + DEAD_ZONE_Y),
                  (0, 255, 255), 1)

    if results[0].boxes:
        box = results[0].boxes[0]
        class_name = results[0].names[int(box.cls[0])]
        confidence = float(box.conf[0])

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        obj_center_x = (x1 + x2) // 2
        obj_center_y = (y1 + y2) // 2

        cv2.circle(annotated, (obj_center_x, obj_center_y), 8, (0, 0, 255), -1)

        if confidence &gt; 0.7:
            command = None
            status  = f&quot;CENTERED - {class_name}&quot;
            color   = (0, 255, 0)

            if cooldown == 0:
                if obj_center_x &lt; CENTER_X - DEAD_ZONE_X:
                    command = &quot;BASE LEFT &quot; + str(TURN_SPEED)
                    status  = &quot;TURNING LEFT&quot;
                    color   = (0, 255, 255)

                elif obj_center_x &gt; CENTER_X + DEAD_ZONE_X:
                    command = &quot;BASE RIGHT &quot; + str(TURN_SPEED)
                    status  = &quot;TURNING RIGHT&quot;
                    color   = (0, 255, 255)

                elif obj_center_y &lt; CENTER_Y - DEAD_ZONE_Y:
                    command = &quot;SHOULDER UP &quot; + str(SHOULDER_SPEED)
                    status  = &quot;SHOULDER UP&quot;
                    color   = (255, 165, 0)

                elif obj_center_y &gt; CENTER_Y + DEAD_ZONE_Y:
                    command = &quot;SHOULDER DOWN &quot; + str(SHOULDER_SPEED)
                    status  = &quot;SHOULDER DOWN&quot;
                    color   = (255, 165, 0)

                if command:
                    try:
                        arduino.write((command + &quot;\n&quot;).encode())
                        print(f&quot;Sent: {command}&quot;)
                    except Exception as e:
                        print(f&quot;Send error: {e}&quot;)
                    cooldown = COOLDOWN_MAX

            cv2.putText(annotated, status,
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, color, 2)
            cv2.putText(annotated, f&quot;{class_name} {confidence:.2f}&quot;,
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 255), 2)

    else:
        cv2.putText(annotated, &quot;No object detected&quot;,
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 0, 255), 2)

    if cooldown &gt; 0:
        cooldown -= 1

    cv2.imshow(&quot;Detection&quot;, annotated)
    if cv2.waitKey(1) == ord(&#x27;q&#x27;):
        break

cap.release()
cv2.destroyAllWindows()</code></pre>
</div>
</details>

---

## [`detect_and_control.py`](../OpenCV/detect_and_control.py)

Early vision-to-Arduino control script. Detects objects and sends <code>BASE LEFT</code>/<code>BASE RIGHT</code> servo commands to pivot the arm toward the target. Predates the DC motor approach — this version turned the base servo instead of driving wheels.

<details>
<summary>Python 3 | 103 lines</summary>
<div class="code-scroll">
<pre><code>import cv2
from ultralytics import YOLO
import serial
import time

model = YOLO(r&quot;C:\Users\leen2\OneDrive\Desktop\pick_place_project\best.pt&quot;)
cap = cv2.VideoCapture(1)

try:
    arduino = serial.Serial(&#x27;COM3&#x27;, 9600)
    time.sleep(2)
    print(&quot;Arduino connected!&quot;)
except:
    print(&quot;Arduino not found, running without it&quot;)
    arduino = None

# Tune these
FRAME_WIDTH  = 640
CENTER_X     = FRAME_WIDTH // 2
DEAD_ZONE    = 50   # pixels — increase if base jitters too much
TURN_SPEED   = 15   # degrees per command — increase for faster turning
COOLDOWN_MAX = 3   # frames between commands — decrease for faster response

cooldown = 0

while True:
    ret, frame = cap.read()
    results = model(frame)
    annotated = results[0].plot()

    # Draw center and dead zone lines
    cv2.line(annotated, (CENTER_X, 0), (CENTER_X, 480), (255, 0, 0), 2)
    cv2.line(annotated, (CENTER_X - DEAD_ZONE, 0),
             (CENTER_X - DEAD_ZONE, 480), (0, 255, 255), 1)
    cv2.line(annotated, (CENTER_X + DEAD_ZONE, 0),
             (CENTER_X + DEAD_ZONE, 480), (0, 255, 255), 1)

    if results[0].boxes:
        box = results[0].boxes[0]
        class_name = results[0].names[int(box.cls[0])]
        confidence = float(box.conf[0])

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        obj_center_x = (x1 + x2) // 2

        # Draw dot at object center
        cv2.circle(annotated, (obj_center_x, (y1 + y2) // 2), 8, (0, 0, 255), -1)

        if confidence &gt; 0.7:
            if cooldown == 0:
                if obj_center_x &lt; CENTER_X - DEAD_ZONE:
                    command = f&quot;BASE LEFT {TURN_SPEED}&quot;
                    status  = &quot;TURNING LEFT&quot;
                    color   = (0, 255, 255)

                elif obj_center_x &gt; CENTER_X + DEAD_ZONE:
                    command = f&quot;BASE RIGHT {TURN_SPEED}&quot;
                    status  = &quot;TURNING RIGHT&quot;
                    color   = (0, 255, 255)

                else:
                    command = None
                    status  = f&quot;CENTERED — {class_name}&quot;
                    color   = (0, 255, 0)

                if command:
                    if arduino:
                        arduino.write(f&quot;{command}\n&quot;.encode())
                    print(f&quot;Sent: {command}&quot;)
                    cooldown = COOLDOWN_MAX

            else:
                if obj_center_x &lt; CENTER_X - DEAD_ZONE:
                    status = &quot;TURNING LEFT&quot;
                    color  = (0, 255, 255)
                elif obj_center_x &gt; CENTER_X + DEAD_ZONE:
                    status = &quot;TURNING RIGHT&quot;
                    color  = (0, 255, 255)
                else:
                    status = f&quot;CENTERED — {class_name}&quot;
                    color  = (0, 255, 0)

            cv2.putText(annotated, status,
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, color, 2)
            cv2.putText(annotated, f&quot;{class_name} {confidence:.2f}&quot;,
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 255), 2)

    else:
        cv2.putText(annotated, &quot;No object detected&quot;,
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 0, 255), 2)

    if cooldown &gt; 0:
        cooldown -= 1

    cv2.imshow(&quot;Detection&quot;, annotated)
    if cv2.waitKey(1) == ord(&#x27;q&#x27;):
        break

cap.release()
cv2.destroyAllWindows()</code></pre>
</div>
</details>

---

## [`detect.py`](../OpenCV/detect.py)

Minimal detection-only script. Loads the YOLO model, opens the camera, and displays annotated frames. No motor control, no tracking — just raw inference. Used for the very first model validation.

<details>
<summary>Python 3 | 16 lines</summary>
<div class="code-scroll">
<pre><code>import cv2
from ultralytics import YOLO

model = YOLO(&quot;best.pt&quot;)
cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    results = model(frame)
    annotated = results[0].plot()
    cv2.imshow(&quot;Detection&quot;, annotated)
    if cv2.waitKey(1) == ord(&#x27;q&#x27;):
        break

cap.release()
cv2.destroyAllWindows()</code></pre>
</div>
</details>

---

## [`capture.py (root)`](../OpenCV/capture.py)

Webcam capture utility with save-to-disk. Press <code>s</code> to save numbered frames. Used to collect the training dataset for the YOLOv11 model.

<details>
<summary>Python 3 | 14 lines</summary>
<div class="code-scroll">
<pre><code>import cv2
cap = cv2.VideoCapture(1)
count = 0
while True:
    ret, frame = cap.read()
    cv2.imshow(&quot;Capture&quot;, frame)
    key = cv2.waitKey(1)
    if key == ord(&#x27;s&#x27;):
        cv2.imwrite(f&quot;image_{count}.jpg&quot;, frame)
        print(f&quot;Saved image_{count}.jpg&quot;)
        count += 1
    elif key == ord(&#x27;q&#x27;):
        break
cap.release()</code></pre>
</div>
</details>

---

## [`capture.py (PICK&PLACE)`](../OpenCV/PICK%26PLACE/capture.py)

Simpler webcam capture script in the deployment directory. Live feed only, no frame saving — used for quick camera checks during integration.

<details>
<summary>Python 3 | 17 lines</summary>
<div class="code-scroll">
<pre><code>import cv2

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        print(&quot;Camera error!&quot;)
        break

    cv2.imshow(&quot;Camera&quot;, frame)

    if cv2.waitKey(1) == ord(&#x27;q&#x27;):
        break

cap.release()
cv2.destroyAllWindows()</code></pre>
</div>
</details>

---

## `model_v2.pt`

> YOLOv11n checkpoint | ~5 MB

The trained model weights. Five classes: Ball, Bolt, Compass, Egg, Screw. Trained for 50 epochs at 640×640 on a Google Colab Tesla T4 GPU using data annotated in Roboflow. This is the same file as `best.pt` in `Final Code/` — renamed for the inference scripts.

---

[← Back to Home](../index.md)
