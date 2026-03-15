# Codebase Review & Task Priorities (2026-03-15)

## Snapshot of current features

- **Command-based WPILib robot** with a CTRE Phoenix 6 **Tuner X swerve drivetrain** (`CommandSwerveDrivetrain`) and PathPlanner integration (AutoBuilder configured in the drivetrain).
- **Turret subsystem** with dual CANcoder CRT seeding, motion-magic control, and Mechanism2d visualization (`TurretSubsystem`).
- **Shooter stack** with dual-flywheel TalonFX leader/follower control, configurable RPM, and telemetry (`FlywheelSubsystem`).
- **Hood subsystem** using Motion Magic, boot-time sensor seeding, soft limits via `HoodAngleUtil`, and simulation support (`HoodSubsystem`).
- **Feeder + intake** using SPARK MAX controllers, plus intake arm motion profiling and simulation (`FeederSubsystem`, `IntakeSubsystem`, `IntakeArmSubsystem`).
- **Auto-aim + shoot** command that calculates turret/hood/flywheel targets based on field pose and velocity (`AutoAimAndShootCommand`, `ShootingCalculator`).
- **Vision integration** in `Robot.robotPeriodic()` with Limelight pose fusion (configurable via `kUseLimelight`).

## Mistakes, warnings, and potential issues

1. **Deprecated REV API usage (build warnings)**
   - `FeederSubsystem` and `IntakeArmSubsystem` use deprecated `SparkBase.ResetMode.kResetSafeParameters` and `SparkBase.PersistMode.kPersistParameters`.
   - `IntakeArmSubsystem` calls a deprecated `SparkClosedLoopController.setReference(...)` overload.
2. **Unused fields**
   - `CommandSwerveDrivetrain` defines `m_sysIdRoutineSteer` and `m_sysIdRoutineRotation` but never uses them.
3. **Unused imports**
   - `LimelightHelpers` imports its own nested classes (`LimelightResults`, `PoseEstimate`) but doesn’t use them.
4. **Unit tests not on classpath**
   - `HoodAngleUtilTest` reports "not on the classpath" in the editor. This might be a Gradle/test source-set issue or a VS Code config mismatch.
5. **Potential logic mismatches**
   - `Robot` seeds the turret CRT in `teleopInit()` only; the CRT comment in `TurretSubsystem` mentions calling on **autonomousInit** too.
   - `kUseLimelight` is set to `true`, while existing project guidance suggests it is normally `false` unless explicitly enabled.
6. **Calculator edge cases**
   - `ShootingCalculator` uses `InterpolatingDoubleTreeMap.get(...)` for time-of-flight; distances outside the map range may return `null` or extrapolate unpredictably.

## Prioritized task list

### P0 — Must address before competition use

1. **Replace deprecated REV APIs** in `FeederSubsystem` and `IntakeArmSubsystem` with the 2026 root-level `com.revrobotics.ResetMode`/`PersistMode` and update the closed-loop `setReference(...)` call to the new signature.
2. **Confirm turret CRT seeding in autonomous** — seed in `autonomousInit()` as well (per turret safety contract).

### P1 — High priority cleanup & reliability

1. **Expose or remove unused SysId routines** in `CommandSwerveDrivetrain` (add selector methods or delete unused fields).
2. **Remove unused imports** from `LimelightHelpers` to keep static analysis clean.
3. **Decide on Limelight enablement**: make `kUseLimelight` a constant in `Constants` or SmartDashboard flag so it’s intentional and traceable.

### P2 — Medium priority hardening

1. **Fix test classpath configuration** so `HoodAngleUtilTest` is recognized by Gradle + VS Code.
2. **Add bounds handling to `ShootingCalculator`** for distances outside of `timeOfFlightMap`/`launchFlywheelSpeedMap` ranges (clamp or default).
3. **Add validation/telemetry** for out-of-range targets (e.g., publish clamped distance or warning on SmartDashboard).

### P3 — Nice-to-have improvements

1. **Add unit tests** for `ShootingCalculator` interpolation and edge cases.
2. **Add basic health checks** (motor connectivity/current alerts) for shooter/feeder/intake on SmartDashboard.

## Notes

- The warnings above are based on current editor diagnostics and static inspection. If you want, I can fix any items directly and run `./gradlew test` afterwards.
