package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.ctre.TalonFXWrapper;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

/**
 * Variable hood subsystem powered by YAMS.
 * Controls a single Kraken X44 motor to adjust the shooter's angle.
 */
public class VariableHoodSubsystem extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final TalonFX hoodMotor = new TalonFX(HOOD_MOTOR_ID, CANBUS);

    // ── YAMS Configuration ────────────────────────────────────────────────────
    private final SmartMotorControllerConfig hoodSmcConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // Real-world TalonFX hardware PID & Feedforward
            .withClosedLoopController(
                HOOD_kP, HOOD_kI, HOOD_kD,
                RotationsPerSecond.of(HOOD_MAX_VEL_DEG_PER_SEC / 360.0),
                RotationsPerSecondPerSecond.of(HOOD_MAX_ACCEL_DEG_PER_SEC2 / 360.0))
            .withFeedforward(new ArmFeedforward(HOOD_kS, HOOD_kG, HOOD_kV))

            // Simulation-specific voltage PID & Feedforward
            .withSimClosedLoopController(
                5.0, 0.0, 0.0,
                RotationsPerSecond.of(HOOD_MAX_VEL_DEG_PER_SEC / 360.0),
                RotationsPerSecondPerSecond.of(HOOD_MAX_ACCEL_DEG_PER_SEC2 / 360.0))
            .withSimFeedforward(new ArmFeedforward(0.0, 0.3, 0.0))

            // Motor Settings
            .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(GearBox.fromRatio(HOOD_GEAR_RATIO)))
            .withMotorInverted(HOOD_MOTOR_INVERTED)
            .withIdleMode(MotorMode.BRAKE)
            .withSupplyCurrentLimit(Amps.of(HOOD_SUPPLY_CURRENT_LIMIT))
            .withStatorCurrentLimit(Amps.of(HOOD_STATOR_CURRENT_LIMIT));

    private final SmartMotorController hoodSmartController =
        new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX60FOC(1), hoodSmcConfig);

    private final ArmConfig hoodArmConfig =
        new ArmConfig(hoodSmartController)
            .withSoftLimits(Degrees.of(HOOD_MIN_ANGLE_DEG), Degrees.of(HOOD_MAX_ANGLE_DEG))
            .withHardLimit(Degrees.of(HOOD_MIN_ANGLE_DEG - 5.0), Degrees.of(HOOD_MAX_ANGLE_DEG + 5.0))
            .withStartingPosition(Degrees.of(HOOD_STARTING_ANGLE_DEG))
            .withLength(Meters.of(HOOD_ARM_LENGTH_METERS))
            .withMass(Kilograms.of(HOOD_ARM_MASS_KG))
            .withTelemetry("Hood", TelemetryVerbosity.HIGH);

    private final Arm hoodArm = new Arm(hoodArmConfig);

    public VariableHoodSubsystem() {
        // Initialization handled fully by YAMS configurations above
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        hoodArm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hoodArm.simIterate();
    }

    // ── Hood Control ──────────────────────────────────────────────────────────

    /** Sets hood to starting angle position. */
    public void setStartingAngle() {
        setAngle(HOOD_STARTING_ANGLE_DEG);
    }

    /** Sets hood to maximum angle position. */
    public void setMaxAngle() {
        setAngle(HOOD_MAX_ANGLE_DEG);
    }

    /** Sets hood to a specific angle in degrees. */
    public void setAngle(double degrees) {
        hoodArm.setMechanismPositionSetpoint(Degrees.of(degrees));
    }

    /**
     * Command factory for moving to a specific target angle.
     * Use this in RobotContainer to easily bind buttons.
     */
    public Command setAngleCommand(double degrees) {
        return hoodArm.run(Degrees.of(degrees));
    }

    /** Stops hood movement immediately. */
    public void stop() {
        hoodSmartController.stopClosedLoopController();
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    /** @return Current hood angle in degrees. */
    public double getAngleDegrees() {
        return hoodSmartController.getMechanismPosition().in(Degrees);
    }

    /** @return true when hood is at target angle within tolerance. */
    public boolean atTargetAngle(double targetDegrees) {
        return Math.abs(getAngleDegrees() - targetDegrees) <= HOOD_AT_TARGET_TOLERANCE_DEG;
    }

    /** @return true when hood is at starting angle within tolerance. */
    public boolean atStartingAngle() {
        return atTargetAngle(HOOD_STARTING_ANGLE_DEG);
    }

    /** @return true when hood is at max angle within tolerance. */
    public boolean atMaxAngle() {
        return atTargetAngle(HOOD_MAX_ANGLE_DEG);
    }
}
