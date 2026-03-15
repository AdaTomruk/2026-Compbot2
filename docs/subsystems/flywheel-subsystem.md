# FlywheelSubsystem

## Overview

`FlywheelSubsystem` controls the shooter flywheel with two Kraken X60 TalonFX motors. The left motor is the leader; the right motor follows in opposite alignment.

## Hardware

- **Left motor (leader):** `LEFT_FLYWHEEL_MOTOR_ID`
- **Right motor (follower):** `RIGHT_FLYWHEEL_MOTOR_ID`
- Both run on the shooter `CANBUS` defined in `Constants.ShooterConstants`.

## Control

- **Velocity control:** `VelocityVoltage` is applied to the left motor; the right motor follows automatically.
- **Follower mode:** `Follower(LEFT_FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed)` handles inversion for the right motor.

### Public API

- `shoot()`: Sets RPM to `STANDARD_SHOOT_RPM`.
- `warmUp()`: Sets RPM to `WARM_UP_RPM`.
- `setRPM(rpm)`: Targets a specific RPM.
- `stop()`: Neutral output (coast).
- `getRPM()`: Average of both motors.
- `atSpeed(targetRPM)` / `atShootSpeed()`

## Configuration details

- Current limits and PIDF values are applied to both motors.
- Sensor ratio (`FLYWHEEL_GEAR_RATIO`) is applied via CTRE Feedback config.

## Telemetry

SmartDashboard outputs:

- left/right/average RPM
- target RPM and error
- supply and stator currents
- boolean `At_Speed` / `At_Shoot_Speed`
