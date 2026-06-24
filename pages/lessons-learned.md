# Lessons Learned

[← Back to Home](../index.md)

---

**Verify component current ratings before integration.** L293D shield shorted under load — chosen for form factor, not for current rating. Lesson: size the driver from the load's stall current, not the convenience.

**Decoupling capacitors are cheap insurance.** A motor blew when a transient knocked the rail and reset the Arduino. Two 2200 µF caps across the servo rail fixed it. Should have been in the original design.

**Separate power rails for different load types.** Pump and servos both pull high transients, but at different times — sharing a rail caused brownouts. Two independent rails fixed it with a small parts cost.

**3D-printing tolerances matter.** Some parts came out wrong-sized from slicer/printer drift and had to be hand-corrected. Build tolerance buffers into the CAD model and print test-fit prototypes before committing to a full run.

**Train detection models on realistic data.** First model trained on clean backgrounds → many false positives in the messy real workspace. Retraining with realistic backgrounds dropped the false positive rate substantially. Collect training data under deployment conditions from the start.

**Scope carefully and be honest about time.** A 4-DOF arm, mobile base, vision, manual interface, digital twin, dual end effector — most worked, not all. A smaller initial scope would have produced a cleaner, fully working system. Scope ambitiously but guarantee the headline feature.

---

[← Back to Home](../index.md)
