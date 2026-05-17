import cv2
import os
import time

SAVE_PATH = r"C:\Users\leen2\OneDrive\Desktop\PICK&PLACE"
CAMERA_INDEX = 1
WIDTH = 1920
HEIGHT = 1080

# Logitech cameras on Windows need DirectShow backend for full property control
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, 30)

# Let auto-exposure run for a moment before locking in
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)   # 0.75 = auto mode on DirectShow
time.sleep(2)                                # warm-up so AE settles

# After warm-up, lock exposure manually with the settled value so it stays stable
# Comment out these two lines if you prefer to leave auto-exposure on
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)   # 0.25 = manual mode
cap.set(cv2.CAP_PROP_EXPOSURE, -6)          # typical good value for Logitech HD; tweak if too dark/bright

cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)       # neutral (0-255)
cap.set(cv2.CAP_PROP_CONTRAST, 128)         # neutral (0-255)
cap.set(cv2.CAP_PROP_SATURATION, 128)       # neutral (0-255)
cap.set(cv2.CAP_PROP_SHARPNESS, 128)        # neutral (0-255)

if not cap.isOpened():
    raise RuntimeError(f"Could not open camera at index {CAMERA_INDEX}. "
                       "Check that the Logitech camera is connected and index is correct.")

os.makedirs(SAVE_PATH, exist_ok=True)

# Determine starting count so we never overwrite existing images
existing = [f for f in os.listdir(SAVE_PATH) if f.startswith("img_") and f.endswith(".jpg")]
count = len(existing)

actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Camera opened at {actual_w}x{actual_h}")
print(f"Saving images to: {SAVE_PATH}")
print("Controls:  [S] save frame   [Q] quit   [A] toggle auto-exposure")

auto_exp = False   # we started in manual mode after warm-up

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame — check camera connection.")
        break

    # Overlay info
    label = f"{actual_w}x{actual_h}  saved: {count}"
    cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow("Logitech Capture — S=save  Q=quit  A=auto-exp", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        filename = os.path.join(SAVE_PATH, f"img_{count:04d}.jpg")
        # Quality 95 keeps detail without inflating file size
        cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        print(f"Saved {filename}")
        count += 1

    elif key == ord('a'):
        auto_exp = not auto_exp
        if auto_exp:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
            print("Auto-exposure ON")
        else:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            cap.set(cv2.CAP_PROP_EXPOSURE, -6)
            print("Auto-exposure OFF (manual -6)")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"Done. {count} images saved to {SAVE_PATH}")
