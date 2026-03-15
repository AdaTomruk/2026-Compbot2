# HoodSubsystem

## Overview

`HoodSubsystem` controls the shooter hood angle with a TalonFX and Motion Magic. It supports seeding to a known angle at boot, soft limits, and simulation of hood motion.

## Hardware

- **Motor:** TalonFX (`HOOD_MOTOR_ID`)
- **Sensor:** Integrated TalonFX sensor with `SensorToMechanismRatio` applied

## Control

- **Motion Magic:** Used for smooth position control with cruise velocity/accel limits.
- **PID enable/disable:** `enablePID()` starts closed-loop control; `disablePID()` neutralizes output.

### Public API

- `setAngle(deg)`: Clamps to min/max and updates target.
- `enablePID()` / `disablePID()`
- `seedPositionDeg(angle)`: Request a seed to a known angle.
- `getAngleDeg()` / `atTarget()` / `isPIDEnabled()`

## Seeding and offsets

- `seedPositionDeg` stores a pending target and offset.
- `applyPendingSeed()` waits 250 ms after boot for CAN stability, then computes the offset using `HoodAngleUtil`.
- Soft limits are recomputed after seeding.

## Telemetry

SmartDashboard outputs:

- target and actual angle
- PID enabled and at-target status

## Simulation

- Uses `SingleJointedArmSim` with a Kraken X44 model.
- Sim state is mapped to raw rotor position and velocity.
- Applies a resting-angle offset so the simulated motor boots at 0 rotations (matching real hardware).
