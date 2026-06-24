# Appendix B — User Manual

[← Back to Home](../index.md)

---

**Powering On.** Plug the Arduino's USB cable into the host. Switch on the servo-rail battery box. Switch on the pump-rail battery box. Visually confirm no smoke or unusual heat before doing anything else.

**Manual Mode.** Open the Processing manual-control sketch on the host. A small window displays the current servo angles. Pair the Bluetooth gamepad. Drive the base with the left stick; shoulder and elbow with the right stick; wrist with the triggers; base rotation with the bumpers.

**Automatic Mode.** Run `detect_and_move.py` on the host. A window opens showing the live camera feed with detections. Place a target object in the workspace. The script detects it and instructs the arm to centre on it. When centred, the script stops and waits for manual pickup. Switch to the gamepad to pick.

**Powering Off.** Switch off the pump rail. Switch off the servo rail. Unplug the Arduino USB.

---

[← Back to Home](../index.md)
