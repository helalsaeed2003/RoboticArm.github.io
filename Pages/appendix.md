# Appendix

[← Back to Home](index.md)

---

## Arduino Serial Servo Controller (Week 4 – Mohamed)

```cpp
// Week 4 - 2-DOF Serial Servo Controller
// Base servo on pin 9, Shoulder servo on pin 10
// Command format: <servo_number> <angle>
// Query: "status" returns current positions
```

*(Full code to be linked or embedded.)*

---

## Camera Detection Code (Leen)

A simple OpenCV-based image capture utility used during development to collect sample images for tuning the color and shape detection pipeline. The script opens a live camera feed and allows the user to save frames on demand.

**Controls:**
- Press `s` — Save the current frame as a `.jpg`
- Press `q` — Quit the application

```python
import cv2

cap = cv2.VideoCapture(1)
count = 0

while True:
    ret, frame = cap.read()
    cv2.imshow("Capture", frame)
    key = cv2.waitKey(1)

    if key == ord('s'):
        cv2.imwrite(f"image_{count}.jpg", frame)
        print(f"Saved image_{count}.jpg")
        count += 1
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

> **Note:** `VideoCapture(1)` selects the external USB camera. Use `0` for the built-in webcam if testing on a laptop.

---

## CAD Model
 ![CAD Model](CAD.PNG) 

