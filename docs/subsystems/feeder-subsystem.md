# FeederSubsystem

## Overview

`FeederSubsystem` drives the funnel/feeder motor that moves game pieces into the shooter.

## Hardware

- **Motor controller:** REV SparkMax (`FEEDER_MOTOR_ID`)
- **Motor type:** Brushless

## Control

- Uses fixed speed constants from `Constants.FeederConstants`.

### Public API

- `runFeeder()`: Runs forward at `FEEDER_SPEED`.
- `reverseFeeder()`: Runs backward for jam clearing.
- `stop()`: Stops output.

## Configuration

- Inversion and smart current limits are configured on boot.
- Settings are persisted on the controller (`PersistMode.kPersistParameters`).

## Telemetry

- No telemetry is currently published; add to `periodic()` if needed.
