# TurretSubsystem

## Overview

`TurretSubsystem` controls a turret driven by a TalonFX with dual CANcoder CRT (common-gear) absolute solving. It provides robot-relative and field-relative aiming, enforces soft limits, and includes a Mechanism2d visualization.

## Hardware

- **Motor:** TalonFX (`TURN_MOTOR_ID`) on the `rio` CAN bus.
- **Sensors:** Dual CANcoders (10T and 17T pinions) for CRT seeding.
- **CRT Solver:** `EasyCRT` combines both encoders to resolve absolute turret angle.

## CRT seeding

- `seedMotorFromCRT()` must be called in `teleopInit` and `autonomousInit` after CAN stabilization.
- Until `crtSeeded == true`, all motion requests are blocked with a warning.
- In simulation, CRT is bypassed and the motor is seeded to 0 rotations.

## Control flow

All movement flows through `driveToRotations(...)`, which enforces:

- CRT seeding gate (blocks motion if unseeded)
- shortest-path rotation
- soft-limit clamping (`MIN_ANGLE_ROT`, `MAX_ANGLE_ROT`)

### Public API

- `setHeading(deg)`: Robot-relative absolute heading (disables chassis compensation).
- `rotateBy(deg)`: Relative rotation from current position.
- `setFieldHeading(deg)`: Locks turret to a field-relative heading with chassis compensation.
- `enableChassisCompensation()` / `disableChassisCompensation()`
- `stop()` / `testOpenLoop(percent)`

## Chassis compensation

When enabled, `periodic()` re-computes the turret’s robot-relative angle based on the chassis heading supplier.

## Telemetry

Published to SmartDashboard:

- mechanism angles, target angles, closed-loop error
- CRT angle and seed status
- current, velocity, chassis heading, field target
- Mechanism2d visualization: turret heading (orange) and target (cyan)

## Simulation

- Uses `DCMotorSim` with Kraken X60 model.
- Writes simulated rotor position/velocity to TalonFX sim state.

## Safety notes

- Soft limits are enforced in motor rotations.
- Movement is blocked until CRT seeding succeeds.
