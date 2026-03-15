# IntakeSubsystem

## Overview

`IntakeSubsystem` controls the intake rollers that acquire and eject game pieces. It uses three SparkMax controllers: upper index, bottom roller, and outer roller.

## Hardware

- **Upper index roller:** `UPPER_INDEX_MOTOR_ID`
- **Bottom roller:** `BOTTOM_ROLLER_MOTOR_ID`
- **Outer roller:** `OUTER_ROLLER_MOTOR_ID`
- All are brushless SparkMax controllers.

## Configuration

Each motor is configured with:

- inversion from `Constants.IntakeConstants`
- brake idle mode
- smart current limits (`ROLLER_SMART_CURRENT_LIMIT_A`)

## Control

### Public API

- `intake()`: Runs all rollers inward using *_INTAKE_OUTPUT constants.
- `outtake()`: Runs all rollers outward using *_OUTTAKE_OUTPUT constants.
- `stopRollers()`: Stops all motors.

## Telemetry

SmartDashboard outputs the motor outputs each loop:

- `Intake/Upper_Output`
- `Intake/Bottom_Output`
- `Intake/Outer_Output`
