import cv2
from ultralytics import YOLO
import serial
import time

model = YOLO(r"C:\Users\Helal\Desktop\School\term 13\Mechatronics\Github\RoboticArm.github.io\OpenCV\PICK&PLACE\model_v2.pt")
cap = cv2.VideoCapture(0)

try:
    arduino = serial.Serial('COM10', 9600, timeout=1)
    time.sleep(2)
    print("Arduino connected!")
except Exception as e:
    print(f"Arduino not found: {e}")
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

        if confidence > 0.7:
            command = None
            status  = f"CENTERED - {class_name}"
            color   = (0, 255, 0)

            if cooldown == 0:
                if obj_center_x < CENTER_X - DEAD_ZONE_X:
                    command = "BASE LEFT " + str(TURN_SPEED)
                    status  = "TURNING LEFT"
                    color   = (0, 255, 255)

                elif obj_center_x > CENTER_X + DEAD_ZONE_X:
                    command = "BASE RIGHT " + str(TURN_SPEED)
                    status  = "TURNING RIGHT"
                    color   = (0, 255, 255)

                elif obj_center_y < CENTER_Y - DEAD_ZONE_Y:
                    command = "SHOULDER UP " + str(SHOULDER_SPEED)
                    status  = "SHOULDER UP"
                    color   = (255, 165, 0)

                elif obj_center_y > CENTER_Y + DEAD_ZONE_Y:
                    command = "SHOULDER DOWN " + str(SHOULDER_SPEED)
                    status  = "SHOULDER DOWN"
                    color   = (255, 165, 0)

                if command:
                    try:
                        arduino.write((command + "\n").encode())
                        print(f"Sent: {command}")
                    except Exception as e:
                        print(f"Send error: {e}")
                    cooldown = COOLDOWN_MAX

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