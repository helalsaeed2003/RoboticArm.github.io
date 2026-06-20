# Control Logic and Programming

[← Back to Home](../index.md)

---

## Firmware Development

The Arduino R4 firmware:
- Receives serial commands from the PC in a combined frame format (`S<sh>,<el>,<wr>,<ha>,M<l>,<r>,P<p>,W<w>`)
- Drives MG996R servos via PWM with constrained 0–180° range
- Runs a **PID control loop** for IMU-based wrist auto-levelling (see below)
- Reads IMU data at 50 Hz for end effector orientation feedback
- Implements a **state-dependent safety interlock** that gates all motor and pump actuation
- Responds to `status`, `cal`, and `pid` queries with current system state
- Initializes all servos to 90° on startup with staggered delays to limit inrush current

## PID Control – Wrist Auto-Levelling

The wrist servo maintains a level orientation using a discrete PID controller driven by the MPU6050 IMU pitch error at 50 Hz. The controller computes:

$$u(t) = K_p \cdot e(t) \;+\; K_i \int_0^t e(\tau)\,d\tau \;+\; K_d \frac{de}{dt}$$

| Parameter | Value | Purpose |
|-----------|-------|---------|
| Kp | 1.0 | Proportional gain — immediate correction proportional to tilt error |
| Ki | 0.05 | Integral gain — eliminates steady-state offset from gravity bias |
| Kd | 0.15 | Derivative gain — damps oscillation during fast arm movements |
| Integral limit | ±30° | Anti-windup clamp prevents integral saturation during sustained errors |
| Update rate | 50 Hz | Matched to IMU read cycle (20 ms period) |

The PID output is added to the 90° neutral position and constrained to [0, 180]° before writing to the servo. The integral term and last-error state are reset on recalibration (`cal` command) to avoid transient kicks.

**Implementation:** `Final Code/ArmController_PID_Safety/ArmController_PID_Safety.ino`, function `pidCompute()`.

## Fuzzy Logic Controller – Vision-Based Motor Speed

The vision-guided motor control uses a **fuzzy logic controller** that maps pixel error (distance from the object centre to the frame centre) to motor PWM speed. This replaces the original fixed-speed bang-bang approach with adaptive speed control — the robot moves fast when the object is far off-centre and slows to a crawl as it approaches the target, reducing overshoot.

### Membership Functions (Input: |pixel error|)

| Linguistic Variable | Shape | Parameters (a, b, c, d) |
|--------------------|-----------|-----------------------|
| Small | Trapezoidal | (−1, 0, 20, 60) |
| Medium | Trapezoidal | (30, 80, 120, 170) |
| Large | Trapezoidal | (120, 200, 300, 301) |

### Rule Base

| Rule | IF error IS ... | THEN speed IS ... | Output Singleton |
|------|----------------|-------------------|-----------------|
| R1 | Small | Slow | 80 PWM |
| R2 | Medium | Medium | 150 PWM |
| R3 | Large | Fast | 230 PWM |

### Defuzzification

Centre-of-gravity (weighted average) defuzzification:

$$\text{speed} = \frac{\mu_S \cdot 80 + \mu_M \cdot 150 + \mu_L \cdot 230}{\mu_S + \mu_M + \mu_L}$$

The Python vision script (`detect_and_move_fuzzy.py`) sends `FUZZY_PIVOT <error>` or `FUZZY_DRIVE <error>` commands with the raw signed pixel offset; the Arduino firmware determines direction from the sign and speed from the fuzzy inference engine.

**Implementation:** `Final Code/PickAndMove_FuzzyLogic/PickAndMove_FuzzyLogic.ino`, functions `fuzzyTrapezoid()` and `fuzzyComputeSpeed()`.

## Safety Interlock (Sequential Logic)

The system implements a state-dependent safety protocol: the arm **shall not initiate movement** unless the Safety Clear signal is asserted. The interlock is evaluated every 50 ms and requires ALL of the following conditions to be true simultaneously:

| Condition | Signal Source | Logic |
|-----------|--------------|-------|
| Hardware interlock | Digital input on pin A0 | Must read HIGH (active-high with pull-down resistor) |
| Serial link alive | Timestamp of last received command | Must be within 2 000 ms of current time |
| Sensor health | IMU detection flag | IMU must be OK, or wrist must be in manual mode |

When the interlock **opens** (any condition fails):
1. Both DC motors are immediately stopped (direction pins LOW, PWM = 0)
2. The vacuum pump relay is de-energized (pin driven HIGH → relay OFF)
3. A `SAFETY: interlock OPEN` message is sent over serial
4. Servo positions are preserved (safe to hold position without power to motors)

When the interlock **clears** (all conditions restored):
1. A `SAFETY: interlock CLEAR` message is sent over serial
2. Subsequent motor/pump commands from the PC are accepted and executed normally

The hardware interlock pin (A0) can be connected to a physical emergency-stop button, a safety relay, or a light-curtain output. During development without external safety hardware, the pin can be tied HIGH to keep the interlock permanently clear.

**Implementation:** `Final Code/ArmController_PID_Safety/ArmController_PID_Safety.ino`, function `updateSafetyInterlock()`.

## Reliability and Safety Mechanisms

### Hardware Watchdog Timer (WDT)

Both firmware variants enable the AVR hardware watchdog timer with a 2-second timeout (`wdt_enable(WDTO_2S)`). The watchdog counter is reset at the top of every `loop()` iteration via `wdt_reset()`. If the main loop ever hangs — due to an I2C bus lockup, infinite loop, or unexpected firmware fault — the watchdog expires and performs a full MCU reset, returning the system to a known-safe state (servos at 90°, motors stopped, pump off).

### XOR Checksum (Parity Check)

The serial protocol supports an optional XOR checksum for error detection. A command frame may include a `*XX` suffix, where `XX` is the two-digit uppercase hex representation of the XOR of all bytes preceding the `*`. On reception, the firmware recomputes the XOR and rejects the frame with `ERR: checksum mismatch` if it does not match.

Example: for the command `S90,90,90,90,M0,0,P0,W1`, the checksum is computed as:
```
XOR of all ASCII bytes = 0x53 ^ 0x39 ^ 0x30 ^ ... = 0xAB
Transmitted frame: S90,90,90,90,M0,0,P0,W1*AB
```

The checksum is backward-compatible — frames without `*XX` are accepted without verification, allowing legacy scripts and manual serial testing to work unchanged.

### Additional Safety Features

| Feature | Description |
|---------|-------------|
| I2C bus timeout | `Wire.setWireTimeout(3000, true)` — 3 ms timeout with automatic bus reset; prevents the IMU from freezing the main loop |
| Graceful degradation | If the IMU is absent at boot, wrist auto-level is disabled but all other subsystems continue operating |
| Servo angle clamping | All servo writes are `constrain()`-ed to [0, 180]° in firmware, regardless of input |
| Pump fail-safe | Relay is active-LOW; loss of power or MCU reset → relay de-energizes → pump stops |
| Pulse-based motion | DC motor commands in the vision sketch auto-expire after 70 ms; a stalled or crashed Python script cannot leave motors running |
| Serial buffer bounds | 64-byte receive buffer with overflow protection; excess characters are silently discarded |

## I/O Interfacing and Industrial Control

| Interface | Protocol | Description |
|-----------|----------|-------------|
| PC ↔ Arduino | USB Serial (9600 baud) | Command/response for joint and motor control |
| Arduino ↔ Servos | PWM | Direct servo position control (0–180°) |
| Arduino ↔ DC Motors | H-Bridge (L298N) | Direction + PWM speed via PID or fuzzy logic |
| Arduino ↔ IMU | I2C (MPU6050) | Pitch feedback for PID wrist auto-levelling |
| Arduino ↔ Safety | Digital GPIO (A0) | Hardware interlock input (active-high) |
| PC ↔ Camera | USB | Video feed for YOLOv8 + OpenCV processing |

## Communication and Networking

- **Primary link:** USB serial between PC and Arduino R4 (9600 baud, newline-terminated frames with optional XOR checksum)
- **ROS2 integration (planned):** ROS2 nodes on the PC will publish joint commands and subscribe to sensor feedback, enabling integration with the Gazebo digital twin for simulation and verification.
