# IntakeArmSubsystem

## Overview

`IntakeArmSubsystem` controls the pivoting intake arm using a SparkMax with a closed-loop position controller. It uses a trapezoidal motion profile and arm feedforward, with simulation support for the arm dynamics.

## Hardware

- **Pivot motor:** SparkMax (`PIVOT_MOTOR_ID`)
- **Encoder:** SparkMax relative encoder

## Control strategy

- **Motion profiling:** `TrapezoidProfile` limits velocity and acceleration.
- **Closed-loop:** SparkMax position control (`kPosition`) with PID gains.
- **Feedforward:** `ArmFeedforward` adds gravity/velocity compensation.

### Public API

- `openArm()` / `closeArm()` / `toggleArm()`
- `getAngleDeg()`
- `atTarget()`
- `getState()` returns `OPEN` or `CLOSE`

## State machine

- `mPeriodicIO` tracks the requested target angle and arm state.
- `isOpen` is a static flag used for toggling and dashboard display.

## Simulation

- Uses `SingleJointedArmSim` with a NEO motor model and `SparkMaxSim`.
- Sim output is fed back into the SparkMax sim on every loop.

## Telemetry

SmartDashboard outputs:

- open/close state
- target angle and current angle
- at-target boolean
