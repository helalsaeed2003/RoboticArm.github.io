# Future Work

[← Back to Home](../index.md)

---

**Complete the automatic pick-and-place cycle.** Camera-frame → workspace-frame coordinate mapping and an inverse-kinematics solver to convert target XYZ into joint angles. A few weeks of focused effort.

**Functional ROS 2 digital twin.** Fix the URDF joint transforms. Lets trajectories be simulated before running on the physical arm.

**Benchmarking against textbook case studies.** Compare against the Bolton industrial pick-and-place example [1, Ch. 24]. The PickMasters prototype uses cheaper hardware, neural-network detection, and fuzzy control — but lacks joint encoders, hurting positional accuracy. Closing that gap means tighter mechanical tolerances and/or encoder feedback.

**Closed-loop DC motor control.** Add Hall-effect or optical encoders + PID firmware to the base motors. Enables precise position control and autonomous waypoint navigation.

**Object-specific grasp selection.** Use the detection class to decide suction vs. claw on a per-object basis.

**Industrial hardening.** Replace Arduino with a PLC, add hardened watchdogs and safety interlocks, and house the electronics in a rated enclosure.

---

[← Back to Home](../index.md)
