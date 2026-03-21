# Calibration & Measurement Guide

This document lists every field in code that requires calibration or real‑world measurement, plus a step‑by‑step procedure for setting each value. The goal is repeatable robot behavior across subsystems.

> **Scope:** values marked with ⚠️ in code, values that encode physical geometry, or values that must be measured to match hardware.

## Before you start

- Bring a notebook for measured values (degrees, meters, kilograms, gear teeth counts).
- Ensure you can safely power the robot with wheels off the floor for motion tests.
- Verify CAN devices are visible in Phoenix Tuner and have the expected firmware.

---

## 1) Drivetrain (CTRE Tuner X swerve)

**File:** `src/main/java/frc/robot/generated/TunerConstants.java`

### A) Steer/Drive PID gains

- **Fields:** `steerGains` and `driveGains` (`kP`, `kI`, `kD`, `kS`, `kV`, `kA`)
- **Why:** Closed‑loop response depends on actual drivetrain mass, friction, and gearing.
- **How to calibrate:**
  1. Use CTRE Tuner X’s built‑in characterization for steer and drive motors.
  2. Start with low `kP` and increase until you see fast response without oscillation.
  3. Add small `kD` only if oscillation is present.
  4. Re‑check after any mechanical change (wheel tread, weight).
- **Acceptance:** Module reaches target angle/speed with minimal overshoot (<5%) and settles quickly.

### B) Measured speed at 12V

- **Field:** `kSpeedAt12Volts`
- **Why:** Used to scale max speed and feedforward.
- **How to calibrate:**
  1. Run the robot at full output on flat carpet.
  2. Measure max steady speed (m/s) using odometry or a field tape measure and stopwatch.
  3. Update `kSpeedAt12Volts` with the measured value.
- **Acceptance:** Odometry‑reported speed matches field measurement within ~5%.

### C) Coupling ratio

- **Field:** `kCoupleRatio`
- **Why:** Models mechanical coupling between steer rotation and drive motor.
- **How to calibrate:**
  1. Follow CTRE Tuner X instructions for coupling ratio measurement.
  2. Rotate the steer module by a known angle and measure drive motor delta.
- **Acceptance:** After applying the ratio, drive velocity no longer drifts during steering.

### D) Gear ratios and wheel radius

- **Fields:** `kDriveGearRatio`, `kSteerGearRatio`, `kWheelRadius`
- **Why:** These are pure geometry values and must match hardware.
- **How to measure:**
  - Count gear teeth or read from BOM and recompute ratio.
  - Measure wheel radius with calipers or diameter/2 (inches or meters).
- **Acceptance:** Odometry distance closely matches tape measure distance.

### E) Module encoder offsets

- **Fields:** `kFrontLeftEncoderOffset`, `kFrontRightEncoderOffset`, `kBackLeftEncoderOffset`, `kBackRightEncoderOffset`
- **Why:** Aligns absolute encoders to true module angle.
- **How to calibrate:**
  1. Physically align all modules to point forward (parallel to chassis X axis).
  2. Read each CANcoder absolute angle in Phoenix Tuner.
  3. Convert to rotations and set the offsets.
- **Acceptance:** When enabled, modules point forward without rotation command.

### F) Module positions and Pigeon ID

- **Fields:** `kFrontLeftXPos`, `kFrontLeftYPos`, etc. and `kPigeonId`
- **Why:** Incorrect geometry or IMU ID breaks odometry.
- **How to measure:**
  - Measure module center positions from robot center.
  - Confirm the Pigeon ID in Phoenix Tuner.
- **Acceptance:** Robot drives straight with no yaw drift; odometry is stable.

---

## 2) Turret subsystem

**Files:** `src/main/java/frc/robot/Constants.java`, `src/main/java/frc/robot/subsystems/TurretSubsystem.java`

### A) CRT encoder offsets

- **Fields:** `TurretConstants.ENCODER_10T_OFFSET_ROT`, `ENCODER_17T_OFFSET_ROT`
- **Why:** CRT solver requires accurate absolute offsets to seed the turret.
- **How to calibrate:**
  1. Set turret to a known mechanical zero (physical reference mark).
  2. Read both CANcoder absolute positions in Phoenix Tuner.
  3. Convert to rotations and set offsets so CRT returns the known angle.
- **Acceptance:** `seedMotorFromCRT()` reports a turret angle that matches the physical zero.

### B) Turret gear parameters

- **Fields:** `TURRET_RING_GEAR_TEETH`, `ENCODER_10T_PINION_TEETH`, `ENCODER_17T_PINION_TEETH`, `MOTOR_TO_MECHANISM_RATIO`
- **Why:** Used in CRT solving and closed‑loop scaling.
- **How to measure:**
  - Count teeth or verify in CAD/BOM.
  - Recompute `MOTOR_TO_MECHANISM_RATIO` based on actual stages.
- **Acceptance:** Mechanism degrees on dashboard match protractor readings.

### C) Soft limits and range

- **Fields:** `MIN_ANGLE_ROT`, `MAX_ANGLE_ROT`
- **Why:** Prevents turret from hitting hard stops.
- **How to calibrate:**
  1. Slowly rotate to each physical stop by hand.
  2. Record absolute rotations at each limit.
  3. Convert to rotations and set min/max.
- **Acceptance:** Commands never drive into hard stops.

### D) PID/FF and Motion Magic

- **Fields:** `kP`, `kI`, `kD`, `kS`, `kV`, `kA`, `TURRET_CRUISE_DEG_PER_SEC`, `TURRET_ACCEL_DEG_PER_SEC2`, `TURRET_JERK_DEG_PER_SEC3`, `MAX_OUTPUT`
- **Why:** Must be tuned for real inertia and friction.
- **How to calibrate:**
  1. Temporarily raise `MAX_OUTPUT` to a safe value (e.g., 0.12–0.2) so the turret can move.
  2. Tune `kP` first for responsiveness; add `kD` only to reduce overshoot.
  3. Tune `kS` until the turret just starts moving at low command.
  4. Tune `kV` using measured steady‑state speed.
  5. Increase cruise/accel/jerk gradually to the desired speed while maintaining stability.
- **Acceptance:** Turret reaches target quickly with no oscillation and steady error <1°.

### E) CRT match tolerance

- **Field:** `CRT_MATCH_TOLERANCE_ROT`
- **Why:** Determines acceptance window for CRT solver agreement.
- **How to calibrate:**
  - Start with a loose tolerance; tighten until seeding fails, then back off slightly.
- **Acceptance:** CRT seeds reliably within 1–2 enable cycles.

---

## 3) Hood subsystem

**Files:** `src/main/java/frc/robot/Constants.java`, `src/main/java/frc/robot/subsystems/HoodSubsystem.java`

### A) Hood gear ratio and sensor scaling

- **Fields:** `HOOD_GEAR_RATIO`, `HOOD_MOTOR_TO_MECHANISM_RATIO`
- **Why:** Converts motor rotations to hood degrees.
- **How to measure:**
  - Use the gearbox/BOM ratio and verify by moving the hood through a known angle and comparing sensor output.
- **Acceptance:** `getAngleDeg()` matches a protractor/angle gauge.

### B) Min/Max/Starting angles

- **Fields:** `HOOD_STARTING_ANGLE_DEG`, `HOOD_MIN_ANGLE_DEG`, `HOOD_MAX_ANGLE_DEG`, `HOOD_MAX_TRAVEL_DEG`
- **Why:** Defines safe range and boot reference.
- **How to calibrate:**
  1. Move hood to physical minimum and maximum.
  2. Measure angles with an angle gauge.
  3. Set starting angle to the boot posture used by the robot.
- **Acceptance:** Soft limits align with actual stops and dashboard values.

### C) Hood seeding offset

- **Field/Method:** `seedPositionDeg()` and `sensorOffsetDeg`
- **Why:** Aligns internal sensor to a known angle after boot.
- **How to calibrate:**
  1. Place hood at a known angle (usually `HOOD_MIN_ANGLE_DEG`).
  2. Call `seedPositionDeg()` with that angle on boot.
  3. Observe the printed offset and verify `getAngleDeg()` matches.
- **Acceptance:** Dashboard angle equals physical angle within 1°.

### D) PID/FF and Motion Magic constraints

- **Fields:** `HOOD_kP`, `HOOD_kD`, `HOOD_kS`, `HOOD_kG`, `HOOD_kV`, `HOOD_MAX_VEL_DEG_PER_SEC`, `HOOD_MAX_ACCEL_DEG_PER_SEC2`
- **Why:** Controls speed/accuracy of the hood.
- **How to calibrate:**
  1. Tune `kS` until the hood just starts moving.
  2. Add `kP` until response is crisp without overshoot.
  3. Add `kD` if oscillation occurs.
  4. Increase velocity/accel limits until the motion is as fast as desired.
- **Acceptance:** Reaches target quickly, no oscillation, <1° error.

### E) Hood physics for sim

- **Fields:** `HOOD_ARM_LENGTH_METERS`, `HOOD_ARM_MASS_KG`
- **Why:** Simulation uses these for physics accuracy.
- **How to measure:**
  - Measure arm length from pivot to center of mass.
  - Weigh the hood assembly.
- **Acceptance:** Simulated motion resembles real‑world motion.

---

## 4) Flywheel subsystem

**File:** `src/main/java/frc/robot/Constants.java`

### A) Gear ratio

- **Field:** `FLYWHEEL_GEAR_RATIO`
- **Why:** Ensures RPM conversion is accurate.
- **How to measure:**
  - Verify mechanical ratio from BOM and update.
- **Acceptance:** Measured flywheel RPM matches motor sensor RPM.

### B) PID/FF gains

- **Fields:** `FLYWHEEL_kP`, `FLYWHEEL_kS`, `FLYWHEEL_kV`, `FLYWHEEL_kA`
- **Why:** Needed for stable velocity control.
- **How to calibrate:**
  1. Characterize motor with CTRE Phoenix or system identification.
  2. Set `kS` and `kV` from characterization.
  3. Increase `kP` until target RPM holds under load.
- **Acceptance:** Flywheel reaches target RPM quickly and holds within tolerance.

### C) Speed tolerances

- **Field:** `FLYWHEEL_AT_SPEED_TOLERANCE_RPM`
- **Why:** Controls when shooter is considered “ready.”
- **How to calibrate:**
  - Set to the maximum error that still yields accurate shots.
- **Acceptance:** Balls land consistently when “ready” is true.

---

## 5) Intake arm (pivot)

**File:** `src/main/java/frc/robot/Constants.java`

### A) Pivot angles and soft limits

- **Fields:** `PIVOT_OPEN_ANGLE_DEG`, `PIVOT_SOFT_MIN_ANGLE_DEG`, `PIVOT_SOFT_MAX_ANGLE_DEG`, `PIVOT_HARD_MIN_ANGLE_DEG`, `PIVOT_HARD_MAX_ANGLE_DEG`
- **Why:** Defines safe motion envelope.
- **How to calibrate:**
  1. Move pivot to each physical stop.
  2. Record angles and set soft limits slightly inside hard stops.
- **Acceptance:** Pivot never contacts hard stops under command.

### B) Pivot gear ratio

- **Field:** `PIVOT_GEAR_RATIO`
- **Why:** Used for encoder conversion.
- **How to measure:**
  - Count teeth or verify from BOM.
- **Acceptance:** Measured arm angle matches encoder angle.

### C) Pivot PID/FF and constraints

- **Fields:** `PIVOT_kP`, `PIVOT_kS`, `PIVOT_kG`, `PIVOT_kV`, `PIVOT_MAX_VEL_DEG_PER_SEC`, `PIVOT_MAX_ACCEL_DEG_PER_SEC2`
- **How to calibrate:**
  1. Tune `kS` so the arm starts moving at low power.
  2. Tune `kP` for position response.
  3. Add `kG` for gravity compensation.
  4. Increase max velocity/accel carefully.
- **Acceptance:** Smooth motion with minimal overshoot.

### D) Arm mass/length (simulation)

- **Fields:** `PIVOT_ARM_LENGTH_METERS`, `PIVOT_ARM_MASS_KG`
- **How to measure:**
  - Measure pivot‑to‑COM distance and weigh the arm.

---

## 6) Shooting field geometry

**File:** `src/main/java/frc/robot/ShootingConstants.java`

### A) Turret offset on robot

- **Field:** `ROBOT_TO_TURRET` (Transform3d)
- **Why:** Used for shoot‑on‑move ballistics and field calculations.
- **How to measure:**
  - Measure turret center offset from robot origin (meters).
  - Use CAD or a tape measure from frame center to turret axis.
- **Acceptance:** Turret field overlays align with actual target direction.

### B) Target coordinates

- **Fields:** `BLUE_TARGET`, `RED_TARGET`
- **Why:** Shooting calculations depend on real field layout.
- **How to calibrate:**
  - Update with official 2026 field coordinates.
- **Acceptance:** Auto‑aim aligns with target at known positions.

---

## 7) Vision / Limelight (optional)

**File:** `src/main/java/frc/robot/LimelightHelpers.java`

If you use fiducial/POI offsets (`setFiducial3DOffset`, `SetFidcuial3DOffset`), ensure the offsets match camera mounting and pipeline settings. These are not currently referenced by robot code but are available for future use.

---

## Validation checklist

- [ ] CRT seeding succeeds and `Turret/CRT_Seeded` is true after enable.
- [ ] Hood seeding prints a reasonable offset and `getAngleDeg()` matches physical angle.
- [ ] Swerve modules point forward at boot without manual correction.
- [ ] Odometry distance aligns with tape measure within 5%.
- [ ] Shooter and hood hold target values within their tolerances.

---

## Reference: key files

- `src/main/java/frc/robot/Constants.java`
- `src/main/java/frc/robot/generated/TunerConstants.java`
- `src/main/java/frc/robot/subsystems/TurretSubsystem.java`
- `src/main/java/frc/robot/subsystems/HoodSubsystem.java`
- `src/main/java/frc/robot/ShootingConstants.java`
