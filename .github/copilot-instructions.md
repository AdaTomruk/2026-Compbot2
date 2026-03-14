# Copilot instructions for 2026-Compbot

## Big picture architecture
- This is a **WPILib command-based** FRC robot project (`TimedRobot` entrypoint in `src/main/java/frc/robot/Robot.java`, command wiring in `RobotContainer`).
- Drivetrain is a **CTRE Phoenix 6 Tuner X generated swerve** stack:
  - Hardware/layout constants in `src/main/java/frc/robot/generated/TunerConstants.java`
  - Team logic wrapper in `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`
- Turret is a custom subsystem with **dual-CANcoder CRT absolute solve** via YAGSL/YAMS `EasyCRT` in `src/main/java/frc/robot/subsystems/TurretSubsystem.java`.
- Autonomous uses **PathPlanner** (`AutoBuilder.buildAutoChooser("Tests")` in `RobotContainer`; assets in `src/main/deploy/pathplanner/`).

## Runtime data flow and boundaries
- `Robot.robotPeriodic()` runs scheduler and optional Limelight vision fusion into drivetrain pose estimator.
- Driver inputs are bound in `RobotContainer.configureBindings()` to swerve requests (`FieldCentric`, brake, point wheels, SysId triggers).
- Turret commands are bound in `RobotContainer.configureTurretBindings()` and call subsystem API (`setHeading`, `rotateBy`, `setFieldHeading`, etc.).
- Keep hardware access inside subsystems; command bindings should call subsystem methods, not configure motor controllers directly.

## Project-specific conventions to preserve
- **Do not hand-edit generated swerve geometry lightly**: `frc.robot.generated.TunerConstants` is generator-owned structure plus tuned values.
- Turret movement must go through `TurretSubsystem.driveToRotations(...)` (private guard path), which enforces:
  - CRT seeding gate (`crtSeeded`)
  - shortest-path wrapping
  - soft-range clamping (`MIN_ANGLE_ROT`, `MAX_ANGLE_ROT`)
- Turret tuning values live in `Constants.TurretConstants`; code comments indicate intentional low-speed safety defaults (`MAX_OUTPUT`, MotionMagic cruise/accel/jerk).
- `kUseLimelight` is currently false in `Robot.java`; don’t assume vision is active unless explicitly enabled.

## Build, test, sim workflows
- Build/test use GradleRIO (`build.gradle`, Java 17):
  - `./gradlew build`
  - `./gradlew test`
- Desktop simulation is enabled (`includeDesktopSupport = true`, `wpi.sim.addGui().defaultEnabled = true`):
  - `./gradlew simulateJava`
- Deploy pathplanner and other runtime assets from `src/main/deploy` via GradleRIO deploy tasks.

## Integration points and external dependencies
- Core libraries: WPILib, CTRE Phoenix 6, PathPlanner (`vendordeps/*.json`, Gradle vendor deps).
- CAN topology split is intentional:
  - Swerve on CANivore (`new CANBus("canivore", ...)` in `TunerConstants`)
  - Turret devices on roboRIO CAN bus (`CANBUS = "rio"` in `Constants.TurretConstants`)
- Replay/logging support exists through CTRE hoot replay in `Robot` (`HootAutoReplay`).

## Editing guidance for AI agents
- When modifying controls, update `RobotContainer` bindings first, then adjust subsystem API only if needed.
- When modifying turret behavior, keep CRT seeding safety intact and preserve simulation fallback (`RobotBase.isSimulation()` branch).
- When modifying auto behavior, keep `AutoBuilder.configure(...)` contract intact in `CommandSwerveDrivetrain` and verify chooser naming matches available autos.
- Prefer minimal, targeted edits; this codebase includes hardware safety assumptions in constants and guard code.
