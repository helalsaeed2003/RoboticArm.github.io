# Appendix C — Software Listings

[← Back to Home](../index.md)

---

The full source for the four programs is in the repository. Reference them via the links below.

- **C.1 ArmController.ino** — Arduino manual-mode firmware. 50 Hz main loop, PID wrist auto-level, safety interlock, XOR-checksum serial protocol.
  [`firmware/ArmController/ArmController.ino`](../firmware/ArmController/ArmController.ino)

- **C.2 PickAndMove.ino** — Arduino automatic-mode firmware. Fuzzy-logic controller, pulse-based motion.
  [`firmware/PickAndMove/PickAndMove.ino`](../firmware/PickAndMove/PickAndMove.ino)

- **C.3 DriveControl.pde** — Processing manual-control sketch. Bluetooth gamepad input → serial commands.
  [`host/processing/DriveControl/DriveControl.pde`](../host/processing/DriveControl/DriveControl.pde)

- **C.4 detect_and_move.py** — Python vision-and-control script. YOLOv11 + OpenCV + serial.
  [`host/python/detect_and_move.py`](../host/python/detect_and_move.py)

---

[← Back to Home](../index.md)
