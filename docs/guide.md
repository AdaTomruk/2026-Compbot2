# Shooter Lookup Tables Calibration Guide

## Overview
Because physical models (like drag and projectile motion) are often inaccurate for foam FRC game pieces, this robot relies on **empirical lookup tables** (Interpolating Maps). 

The `ShootingCalculator` uses three primary maps to control the shooter:
1. **`launchFlywheelSpeedMap`**: Maps Distance (meters) to Flywheel Speed (RPM).
2. **`launchHoodAngleMap`**: Maps Distance (meters) to Hood Angle (Degrees).
3. **`timeOfFlightMap`**: Maps Distance (meters) to Time of Flight (Seconds).

These tables must be tuned manually. The `InterpolatingTreeMap` will automatically handle the math for any distances that fall *between* your tuned data points.

---

## Phase 1: Prerequisites
Before collecting a single data point, ensure your robot's baseline is stable. If your hardware is inconsistent, your software will be chasing a moving target.

* **Sorted Game Pieces:** Use a mix of fresh, stiff game pieces and worn-out, squishy ones during tuning. This ensures your averages represent what you'll actually see on the field.
* **Target Alignment:** Ensure your Limelight/Odometry accurately reports the distance to the target. Use a physical tape measure to verify that when the robot thinks it is `2.0` meters away, the center of the turret is actually `2.0` meters away.
* **Shooting on the Move:** **DISABLE** the shoot-on-the-move command for Phase 2. The robot must be completely stationary when mapping RPM and Hood angles.

---

## Phase 2: Static Tuning (Flywheel & Hood)
This phase populates `launchFlywheelSpeedMap` and `launchHoodAngleMap`. 

### The "Flat RPM" Strategy
Heavy flywheels take time to spin up and slow down. To ensure the robot can shoot instantly while driving, aim for a **"flat" RPM curve**. Keep the Flywheel RPM constant across large zones and let the lightweight, fast-moving hood make the micro-adjustments.

### Step-by-Step Tuning
1.  **Mark Distances:** Lay a tape measure on the floor from the base of the target. Mark increments every ~0.5 meters (e.g., 1.5m, 2.0m, 2.5m... up to your max range).
2.  **Position Robot:** Place the center of the robot's turret exactly on your first mark (e.g., 1.5m).
3.  **Find the "Swish":** Using a dashboard (like Glass or Shuffleboard), manually control the Flywheel RPM and Hood Angle. Adjust them until the game piece hits the dead-center of the target reliably.
4.  **Record Data:** Write down the exact `Distance`, `RPM`, and `Hood Angle`.
5.  **Repeat:** Move the robot back to the next mark and repeat the process. Try to keep the RPM identical to the previous distance if possible, only changing the Hood Angle.
6.  **Update Code:** Enter these pairs into the `ShootingCalculator` static block.

> **CRITICAL RULE:** The keys (Distances) in your `launchFlywheelSpeedMap` and `launchHoodAngleMap` **must be identical**. If you tuned the hood at 2.0m, you must have a flywheel data point at exactly 2.0m. Do not tune them independently.

---

## Phase 3: Time of Flight Tuning (The Physics)
To shoot on the move, the 20-iteration lookahead loop needs to know exactly how long the game piece will be in the air. This populates the `timeOfFlightMap`.

### Step-by-Step Tuning
1.  **Use a Camera:** Have a team member stand to the side of the field with a smartphone recording video at **60 Frames Per Second (FPS)**.
2.  **Film the Shots:** Place the robot at your marked distances. Fire a shot using your newly tuned static values (RPM and Hood Angle) while filming the trajectory.
3.  **Count Frames:** In a video editor or player, scrub through the footage frame-by-frame. Count the exact number of frames from the instant the game piece leaves the shooter wheels to the instant it enters the goal.
4.  **Calculate Seconds:** Divide the frame count by 60. 
    * *Example: 54 frames / 60 fps = 0.90 seconds.*
5.  **Update Code:** Map the physical distance to this calculated time in `timeOfFlightMap`.

---

## Phase 4: Code Implementation
Once you have your data, update the static block in `ShootingCalculator.java`:

```java
static {
    // Distance (meters) -> Hood Angle
    launchHoodAngleMap.put(1.50, Rotation2d.fromDegrees(20.0));
    launchHoodAngleMap.put(2.00, Rotation2d.fromDegrees(25.0));
    launchHoodAngleMap.put(3.00, Rotation2d.fromDegrees(32.0));
    // ...

    // Distance (meters) -> Flywheel RPM (Notice keys match the Hood map!)
    launchFlywheelSpeedMap.put(1.50, 2500.0);
    launchFlywheelSpeedMap.put(2.00, 2500.0);
    launchFlywheelSpeedMap.put(3.00, 3000.0);
    // ...

    // Distance (meters) -> Time of flight (seconds)
    timeOfFlightMap.put(1.50, 0.90);
    timeOfFlightMap.put(2.00, 0.95);
    timeOfFlightMap.put(3.00, 1.10);
    // ...
}
