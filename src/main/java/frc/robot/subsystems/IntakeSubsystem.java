package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

/**
 * Intake mechanism subsystem using four SPARK MAX motor controllers:
 * <ul>
 *   <li>31: NEO Vortex upper indexing roller</li>
 *   <li>32: NEO Vortex lower roller/mecanum stage</li>
 *   <li>33: NEO 1.1 pivot deployment motor (YAMS Arm closed loop)</li>
 *   <li>34: NEO 1.1 outer roller</li>
 * </ul>
 *
 * <p>The pivot is controlled via YAMS {@link Arm}, which provides proper simulation
 * physics via {@code simIterate()} — replacing the previous SPARK MAX onboard PID
 * that caused instantaneous position jumps in desktop simulation.</p>
 *
 * <p>Boot-time zero assumption: robot starts physically in the "Closed" intake pose.
 * YAMS seeds the encoder to {@code PIVOT_CLOSED_ANGLE_DEG} via
 * {@code ArmConfig.withStartingPosition}.</p>
 */
public class IntakeSubsystem extends SubsystemBase {

    // ── Roller motors ─────────────────────────────────────────────────────────
    private final SparkMax upperIndexMotor   = new SparkMax(UPPER_INDEX_MOTOR_ID,  MotorType.kBrushless);
    private final SparkMax bottomRollerMotor = new SparkMax(BOTTOM_ROLLER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax outerRollerMotor  = new SparkMax(OUTER_ROLLER_MOTOR_ID,  MotorType.kBrushless);

    // ── Pivot motor (raw SparkMax kept for roller-style stop only) ────────────
    private final SparkMax pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);

    // ── YAMS pivot control ────────────────────────────────────────────────────
    // Field init order matters: pivotMotor → pivotSmcConfig → pivotSmartController
    //                           → pivotArmConfig → pivotArm
    private final SmartMotorControllerConfig pivotSmcConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                PIVOT_kP, PIVOT_kI, PIVOT_kD,
                RotationsPerSecond.of(PIVOT_MAX_VEL_DEG_PER_SEC / 360.0),
                RotationsPerSecondPerSecond.of(PIVOT_MAX_ACCEL_DEG_PER_SEC2 / 360.0))
            .withSimClosedLoopController(
                PIVOT_kP, PIVOT_kI, PIVOT_kD,
                RotationsPerSecond.of(PIVOT_MAX_VEL_DEG_PER_SEC / 360.0),
                RotationsPerSecondPerSecond.of(PIVOT_MAX_ACCEL_DEG_PER_SEC2 / 360.0))
            .withFeedforward(new ArmFeedforward(PIVOT_kS, PIVOT_kG, PIVOT_kV))
            .withSimFeedforward(new ArmFeedforward(PIVOT_kS, PIVOT_kG, PIVOT_kV))
            .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(GearBox.fromStages("15:1", "24:15")))
            .withMotorInverted(PIVOT_INVERTED)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(PIVOT_SMART_CURRENT_LIMIT_A));

    private final SmartMotorController pivotSmartController =
        new SparkWrapper(pivotMotor, DCMotor.getNEO(1), pivotSmcConfig);

    private final ArmConfig pivotArmConfig =
        new ArmConfig(pivotSmartController)
            .withSoftLimits(Degrees.of(PIVOT_SOFT_MIN_ANGLE_DEG), Degrees.of(PIVOT_SOFT_MAX_ANGLE_DEG))
            .withHardLimit(Degrees.of(PIVOT_HARD_MIN_ANGLE_DEG), Degrees.of(PIVOT_HARD_MAX_ANGLE_DEG))
            .withStartingPosition(Degrees.of(PIVOT_CLOSED_ANGLE_DEG))
            .withLength(Meters.of(PIVOT_ARM_LENGTH_METERS))
            .withMass(Kilograms.of(PIVOT_ARM_MASS_KG))
            .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

    private final Arm pivotArm = new Arm(pivotArmConfig);

    public IntakeSubsystem() {
        configureRoller(upperIndexMotor,   UPPER_INDEX_INVERTED);
        configureRoller(bottomRollerMotor, BOTTOM_ROLLER_INVERTED);
        configureRoller(outerRollerMotor,  OUTER_ROLLER_INVERTED);
        // Pivot configured entirely by YAMS via pivotSmcConfig / pivotArmConfig.
        // No separate configurePivot() or setPosition() call needed.
    }

    private void configureRoller(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(inverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ROLLER_SMART_CURRENT_LIMIT_A);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        pivotArm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        pivotArm.simIterate();
    }

    // ── Roller control ────────────────────────────────────────────────────────

    /** Runs all intake rollers inward to acquire game pieces. */
    public void intake() {
        upperIndexMotor.set(UPPER_INDEX_INTAKE_OUTPUT);
        bottomRollerMotor.set(BOTTOM_ROLLER_INTAKE_OUTPUT);
        outerRollerMotor.set(OUTER_ROLLER_INTAKE_OUTPUT);
    }

    /** Runs all intake rollers outward to eject game pieces. */
    public void outtake() {
        upperIndexMotor.set(UPPER_INDEX_OUTTAKE_OUTPUT);
        bottomRollerMotor.set(BOTTOM_ROLLER_OUTTAKE_OUTPUT);
        outerRollerMotor.set(OUTER_ROLLER_OUTTAKE_OUTPUT);
    }

    /** Stops all roller motors. */
    public void stopRollers() {
        upperIndexMotor.stopMotor();
        bottomRollerMotor.stopMotor();
        outerRollerMotor.stopMotor();
    }

    // ── Pivot control ─────────────────────────────────────────────────────────

    /** Commands pivot to configured open position using YAMS Arm closed-loop. */
    public void open() {
        pivotArm.setMechanismPositionSetpoint(Degrees.of(PIVOT_OPEN_ANGLE_DEG));
    }

    /** Commands pivot to configured closed position. */
    public void close() {
        pivotArm.setMechanismPositionSetpoint(Degrees.of(PIVOT_CLOSED_ANGLE_DEG));
    }

    /** Stops pivot output immediately (emergency stop). */
    public void stopPivot() {
        pivotSmartController.stopClosedLoopController();
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    /** @return pivot position in motor rotations (raw encoder). */
    public double getPivotPositionRot() {
        return pivotMotor.getEncoder().getPosition();
    }

    /** @return true when pivot is within tolerance of open setpoint. */
    public boolean isOpen() {
        double positionDeg = pivotSmartController.getMechanismPosition().in(Degrees);
        return Math.abs(positionDeg - PIVOT_OPEN_ANGLE_DEG) <= PIVOT_AT_TARGET_TOLERANCE_DEG;
    }

    /** @return true when pivot is within tolerance of closed setpoint. */
    public boolean isClosed() {
        double positionDeg = pivotSmartController.getMechanismPosition().in(Degrees);
        return Math.abs(positionDeg - PIVOT_CLOSED_ANGLE_DEG) <= PIVOT_AT_TARGET_TOLERANCE_DEG;
    }
}
