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