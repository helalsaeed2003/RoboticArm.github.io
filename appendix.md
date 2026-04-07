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

## CAD Model

*(Link to Inventor files or embed screenshots.)*

<!-- Example: ![CAD Model](assets/cad_model.png) -->

## Camera Detection Code
//import cv2
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
