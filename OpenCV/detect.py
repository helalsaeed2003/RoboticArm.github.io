import cv2
from ultralytics import YOLO
import serial
import time

model = YOLO(r"C:\Users\leen2\OneDrive\Desktop\pick_place_project\best.pt")
cap = cv2.VideoCapture(1)

try:
    arduino = serial.Serial('COM3', 9600)
    time.sleep(2)
    print("Arduino connected!")
except:
    print("Arduino not found, running without it")
    arduino = None

# Tune these
FRAME_WIDTH  = 640
CENTER_X     = FRAME_WIDTH // 2
DEAD_ZONE    = 50   # pixels — increase if base jitters too much
TURN_SPEED   = 30    # degrees per command — increase for faster turning
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

        if confidence > 0.7:
            if cooldown == 0:
                if obj_center_x < CENTER_X - DEAD_ZONE:
                    command = f"BASE LEFT {TURN_SPEED}"
                    status  = "TURNING LEFT"
                    color   = (0, 255, 255)

                elif obj_center_x > CENTER_X + DEAD_ZONE:
                    command = f"BASE RIGHT {TURN_SPEED}"
                    status  = "TURNING RIGHT"
                    color   = (0, 255, 255)

                else:
                    command = None
                    status  = f"CENTERED — {class_name}"
                    color   = (0, 255, 0)

                if command:
                    if arduino:
                        arduino.write(f"{command}\n".encode())
                    print(f"Sent: {command}")
                    cooldown = COOLDOWN_MAX

            else:
                if obj_center_x < CENTER_X - DEAD_ZONE:
                    status = "TURNING LEFT"
                    color  = (0, 255, 255)
                elif obj_center_x > CENTER_X + DEAD_ZONE:
                    status = "TURNING RIGHT"
                    color  = (0, 255, 255)
                else:
                    status = f"CENTERED — {class_name}"
                    color  = (0, 255, 0)

            cv2.putText(annotated, status,
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, color, 2)
            cv2.putText(annotated, f"{class_name} {confidence:.2f}",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 255), 2)

    else:
        cv2.putText(annotated, "No object detected",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 0, 255), 2)

    if cooldown > 0:
        cooldown -= 1

    cv2.imshow("Detection", annotated)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()