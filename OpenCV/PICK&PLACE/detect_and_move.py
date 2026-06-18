"""
detect_and_move.py
==================

Vision + control loop for the PickMasters arm.

The camera feed is split by ONE vertical and ONE horizontal line.  Where the two
lines cross there is a target BOX.  The job of this program is to drive the robot
(DC motors only) until the chosen item sits completely inside that box:

  * VERTICAL line   -> handled by SPINNING the base (pivot the wheels left/right)
                       so the item lines up on the vertical (X) axis.
  * HORIZONTAL line -> handled by MOVING the base forward/back (drive both wheels)
                       so the item lines up on the horizontal (Y) axis.

Locking behaviour
-----------------
The program will only LOCK onto an item once it is seen with >= 80% confidence.
After that it stays focused on the SAME item and keeps centering it, even if the
confidence drops, until you press the "next" key (n).  Then it releases the lock
and is free to pick the next >= 80% item.

Keys
----
  n : release the current lock and look for the next item
  s : send an emergency STOP to the Arduino
  q : quit

Serial protocol (sent to the Arduino, one command per line)
-----------------------------------------------------------
  PIVOT_LEFT  <speed>   spin base left
  PIVOT_RIGHT <speed>   spin base right
  DRIVE_FWD   <speed>   move base forward
  DRIVE_BACK  <speed>   move base backward
  STOP                  stop all DC motors
"""

import os
import cv2
from ultralytics import YOLO
import serial
import time

# --------------------------------------------------------------------------- #
#  Configuration
# --------------------------------------------------------------------------- #
SERIAL_PORT = "COM10"          # change to match your Arduino port
BAUD_RATE   = 9600

# Locate model_v2.pt next to this script so the path works on any machine.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "model_v2.pt")

CAMERA_INDEX = 0

FRAME_WIDTH  = 640
FRAME_HEIGHT = 480
CENTER_X     = FRAME_WIDTH  // 2
CENTER_Y     = FRAME_HEIGHT // 2

# Half-size of the target box drawn at the crossing of the two lines.
# The item must fit COMPLETELY inside this box.
BOX_HALF_W = 90
BOX_HALF_H = 90

BOX_LEFT   = CENTER_X - BOX_HALF_W
BOX_RIGHT  = CENTER_X + BOX_HALF_W
BOX_TOP    = CENTER_Y - BOX_HALF_H
BOX_BOTTOM = CENTER_Y + BOX_HALF_H

# How close (in pixels) the item centre must be to a line before we stop nudging
# on that axis.  Keep this smaller than the box so the item ends up well inside.
DEAD_ZONE_X = 30
DEAD_ZONE_Y = 30

LOCK_CONFIDENCE = 0.80         # must reach this to LOCK onto an item

PIVOT_SPEED = 120              # PWM (0-255) the Arduino uses while spinning base
DRIVE_SPEED = 120              # PWM (0-255) the Arduino uses while driving base

COOLDOWN_MAX = 3               # frames to wait between motion pulses


# --------------------------------------------------------------------------- #
#  Setup
# --------------------------------------------------------------------------- #
model = YOLO(MODEL_PATH)
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("Arduino connected!")
except Exception as e:
    print(f"Arduino not found: {e}")
    arduino = None


def send(command):
    """Send a single line command to the Arduino (and echo to the console)."""
    print(f"Sent: {command}")
    if arduino:
        try:
            arduino.write((command + "\n").encode())
        except Exception as e:
            print(f"Send error: {e}")


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

    # ---- Acquire a lock if we don't have one ------------------------------ #
    if locked_id is None or locked_id not in tracked:
        # Look for the highest-confidence item that clears the 80% threshold.
        best_id, best_conf = None, 0.0
        for tid, box in tracked.items():
            conf = float(box.conf[0])
            if conf >= LOCK_CONFIDENCE and conf > best_conf:
                best_id, best_conf = tid, conf

        if best_id is not None:
            locked_id = best_id
            print(f"LOCKED onto item id {locked_id} ({best_conf:.2f})")
        else:
            # Nothing to lock onto -> make sure the robot is stopped.
            if cooldown == 0:
                send("STOP")
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
        fully_inside = (x1 >= BOX_LEFT and x2 <= BOX_RIGHT and
                        y1 >= BOX_TOP  and y2 <= BOX_BOTTOM)

        command = None
        if fully_inside:
            status, color = f"READY - {class_name} in box", (0, 255, 0)
        elif obj_cx < CENTER_X - DEAD_ZONE_X:
            # Item is left of the vertical line -> spin base left.
            command = f"PIVOT_LEFT {PIVOT_SPEED}"
            status, color = "SPIN BASE LEFT", (0, 255, 255)
        elif obj_cx > CENTER_X + DEAD_ZONE_X:
            command = f"PIVOT_RIGHT {PIVOT_SPEED}"
            status, color = "SPIN BASE RIGHT", (0, 255, 255)
        elif obj_cy < CENTER_Y - DEAD_ZONE_Y:
            # Item is above the horizontal line -> move base forward.
            command = f"DRIVE_FWD {DRIVE_SPEED}"
            status, color = "MOVE BASE FORWARD", (255, 165, 0)
        elif obj_cy > CENTER_Y + DEAD_ZONE_Y:
            command = f"DRIVE_BACK {DRIVE_SPEED}"
            status, color = "MOVE BASE BACK", (255, 165, 0)
        else:
            # Centre is on the cross but the box isn't fully contained yet
            # (item bigger than the dead zone) -> nudge it the rest of the way.
            command = None
            status, color = "CENTERING...", (0, 255, 0)

        if cooldown == 0:
            send(command if command else "STOP")
            cooldown = COOLDOWN_MAX

        cv2.putText(annotated, status, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(annotated, f"LOCK id{locked_id} {class_name} {confidence:.2f}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    elif locked_id is not None:
        # We have a lock but lost sight of it this frame -> hold position.
        if cooldown == 0:
            send("STOP")
            cooldown = COOLDOWN_MAX
        cv2.putText(annotated, f"SEARCHING for locked id {locked_id}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    else:
        cv2.putText(annotated, "No >=80% item to lock onto",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    if cooldown > 0:
        cooldown -= 1

    cv2.imshow("Pick & Move", annotated)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('n'):
        # Release the lock so the next >=80% item can be chosen.
        print(f"Released lock on id {locked_id}")
        locked_id = None
        send("STOP")
    elif key == ord('s'):
        send("STOP")

send("STOP")
cap.release()
cv2.destroyAllWindows()
if arduino:
    arduino.close()
