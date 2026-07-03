"""
Logitech HD Camera Capture Tool
Captures labelled training images at 1080p to a structured folder.

Controls
--------
  S          Save current frame
  N          New label  (creates a subfolder, resets counter)
  [ / ]      Decrease / Increase exposure  (one step at a time)
  A          Toggle auto-exposure on/off
  Q          Quit
"""

import cv2
import os
import time

# ─── Configuration ────────────────────────────────────────────────────────────

CAMERA_INDEX = 1                                          # Logitech = 1
BASE_SAVE_PATH = r"C:\Users\leen2\OneDrive\Desktop\PICK&PLACE"
JPEG_QUALITY   = 95                                       # 0-100; 95 = high quality, reasonable size
TARGET_W       = 1920
TARGET_H       = 1080
TARGET_FPS     = 30
WARMUP_FRAMES  = 40                                       # frames to skip while auto-exposure settles

# Logitech exposure range on Windows DirectShow is roughly -13 (very dark) to -1 (very bright)
# -6 works well for a typical indoor desktop environment
DEFAULT_EXPOSURE = -6

# ─── Camera initialisation ────────────────────────────────────────────────────

def open_camera(index: int) -> cv2.VideoCapture:
    # CAP_DSHOW is required on Windows to actually write camera properties
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError(
            f"Cannot open camera at index {index}.\n"
            "  • Make sure the Logitech camera is plugged in.\n"
            "  • If it is the only camera, try index 0 instead of 1."
        )
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  TARGET_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_H)
    cap.set(cv2.CAP_PROP_FPS,          TARGET_FPS)
    return cap


def warmup_and_lock_exposure(cap: cv2.VideoCapture) -> int:
    """Let auto-exposure settle over WARMUP_FRAMES, then lock to manual."""
    # Enable auto-exposure while warming up
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)   # 0.75 = auto on DirectShow

    print(f"Warming up camera ({WARMUP_FRAMES} frames) …", end="", flush=True)
    for _ in range(WARMUP_FRAMES):
        cap.read()
    print(" done.")

    # Lock to manual so brightness stays consistent across every training image
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)   # 0.25 = manual on DirectShow
    cap.set(cv2.CAP_PROP_EXPOSURE, DEFAULT_EXPOSURE)

    # Neutral image quality settings (scale depends on camera; 128 is mid-point for Logitech)
    cap.set(cv2.CAP_PROP_BRIGHTNESS,  128)
    cap.set(cv2.CAP_PROP_CONTRAST,    128)
    cap.set(cv2.CAP_PROP_SATURATION,  140)       # slightly richer colour helps detection models
    cap.set(cv2.CAP_PROP_SHARPNESS,   150)       # a little extra sharpness aids edge detection

    return DEFAULT_EXPOSURE


# ─── Helpers ─────────────────────────────────────────────────────────────────

def make_save_dir(base: str, label: str) -> str:
    path = os.path.join(base, label)
    os.makedirs(path, exist_ok=True)
    return path


def next_index(folder: str) -> int:
    """Find the highest existing image index so we never overwrite anything."""
    existing = [
        f for f in os.listdir(folder)
        if f.startswith("img_") and f.endswith(".jpg")
    ]
    if not existing:
        return 0
    indices = []
    for f in existing:
        try:
            indices.append(int(f[4:8]))
        except ValueError:
            pass
    return max(indices) + 1 if indices else 0


def brightness_warning(frame) -> str:
    """Return a warning string if the frame is over- or under-exposed."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mean = gray.mean()
    if mean < 40:
        return "UNDEREXPOSED  press ] to increase exposure"
    if mean > 215:
        return "OVEREXPOSED   press [ to decrease exposure"
    return ""


def draw_hud(frame, label: str, count: int, exposure: int, auto: bool, warning: str):
    h, w = frame.shape[:2]

    # Semi-transparent dark bar at the top
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, 52), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

    exp_str  = "AUTO" if auto else str(exposure)
    info     = f"Label: {label}   Saved: {count}   Exposure: {exp_str}   {w}x{h}"
    controls = "S=save  N=new label  [/]=exposure  A=auto-exp  Q=quit"

    cv2.putText(frame, info,     (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0),  1, cv2.LINE_AA)
    cv2.putText(frame, controls, (10, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1, cv2.LINE_AA)

    if warning:
        cv2.putText(frame, warning, (10, h - 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 80, 255), 2, cv2.LINE_AA)


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    # Ask for initial label
    label = input("Enter label / class name for this session (e.g. cube, empty): ").strip()
    if not label:
        label = "default"

    cap      = open_camera(CAMERA_INDEX)
    exposure = warmup_and_lock_exposure(cap)
    auto_exp = False

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Resolution : {actual_w} x {actual_h}")
    print(f"Save path  : {BASE_SAVE_PATH}")
    print(f"Label      : {label}")
    print("─" * 50)

    save_dir = make_save_dir(BASE_SAVE_PATH, label)
    count    = next_index(save_dir)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame grab failed — check the camera connection.")
            break

        warning = brightness_warning(frame)
        draw_hud(frame, label, count, exposure, auto_exp, warning)

        cv2.imshow("Logitech Capture", frame)
        key = cv2.waitKey(1) & 0xFF

        # ── Save ──────────────────────────────────────────────────────────────
        if key == ord('s'):
            filename = os.path.join(save_dir, f"img_{count:04d}.jpg")
            # Save a clean frame (re-read so the HUD overlay is NOT baked into the image)
            ret2, clean = cap.read()
            if ret2:
                cv2.imwrite(filename, clean, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                print(f"  Saved: {filename}")
                count += 1
            else:
                cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                print(f"  Saved (display frame): {filename}")
                count += 1

        # ── New label ─────────────────────────────────────────────────────────
        elif key == ord('n'):
            cv2.destroyAllWindows()
            new_label = input("New label: ").strip()
            if new_label:
                label    = new_label
                save_dir = make_save_dir(BASE_SAVE_PATH, label)
                count    = next_index(save_dir)
                print(f"Switched to label '{label}' — save dir: {save_dir}")

        # ── Decrease exposure ─────────────────────────────────────────────────
        elif key == ord('['):
            if not auto_exp:
                exposure = max(exposure - 1, -13)
                cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
                print(f"  Exposure → {exposure}")

        # ── Increase exposure ─────────────────────────────────────────────────
        elif key == ord(']'):
            if not auto_exp:
                exposure = min(exposure + 1, -1)
                cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
                print(f"  Exposure → {exposure}")

        # ── Toggle auto-exposure ───────────────────────────────────────────────
        elif key == ord('a'):
            auto_exp = not auto_exp
            if auto_exp:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
                print("  Auto-exposure ON")
            else:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
                cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
                print(f"  Auto-exposure OFF  (locked at {exposure})")

        # ── Quit ──────────────────────────────────────────────────────────────
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"\nSession complete. {count} images saved under '{label}' in {BASE_SAVE_PATH}")


if __name__ == "__main__":
    main()
