package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.Constants.IntakeConstants.*; // Assuming pivot constants are still here

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

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
import yams.motorcontrollers.local.SparkWrapper;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

/**
 * Pivot mechanism subsystem powered by YAMS.
 * Uses a single NEO motor to control the arm's angle.
 */
public class PivotSubsystem extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final SparkMax pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);

    // ── YAMS Configuration ────────────────────────────────────────────────────
    private final SmartMotorControllerConfig pivotSmcConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // Real-world Spark Max hardware PID & Feedforward
            .withClosedLoopController(
                PIVOT_kP, PIVOT_kI, PIVOT_kD,
                RotationsPerSecond.of(PIVOT_MAX_VEL_DEG_PER_SEC / 360.0),
                RotationsPerSecondPerSecond.of(PIVOT_MAX_ACCEL_DEG_PER_SEC2 / 360.0))
            .withFeedforward(new ArmFeedforward(PIVOT_kS, PIVOT_kG, PIVOT_kV))
            
            // Simulation-specific voltage PID & Feedforward
            .withSimClosedLoopController(
                0.5, 0.0, 0.0, 
                RotationsPerSecond.of(PIVOT_MAX_VEL_DEG_PER_SEC / 360.0),
                RotationsPerSecondPerSecond.of(PIVOT_MAX_ACCEL_DEG_PER_SEC2 / 360.0))
            .withSimFeedforward(new ArmFeedforward(0.0, 0.5, 0.0))
            
            // Motor Settings
            .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(GearBox.fromStages("15:1", "24:15")))
            .withMotorInverted(PIVOT_INVERTED)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(PIVOT_SMART_CURRENT_LIMIT_A));

    private final SmartMotorController pivotSmartController =
        new SparkWrapper(pivotMotor, DCMotor.getNEO(1), pivotSmcConfig);

    private final ArmConfig pivotArmConfig =
        new ArmConfig(pivotSmartController)
            .withSoftLimits(Degrees.of(PIVOT_SOFT_MIN_ANGLE_DEG), Degrees.of(PIVOT_SOFT_MAX_ANGLE_DEG))
            // Widened sim hard limits to prevent immediate wall-collisions in simulation
            .withHardLimit(Degrees.of(PIVOT_HARD_MIN_ANGLE_DEG - 5.0), Degrees.of(PIVOT_HARD_MAX_ANGLE_DEG + 5.0))
            .withStartingPosition(Degrees.of(PIVOT_CLOSED_ANGLE_DEG))
            .withLength(Meters.of(PIVOT_ARM_LENGTH_METERS))
            .withMass(Kilograms.of(PIVOT_ARM_MASS_KG))
            .withTelemetry("PivotArm", TelemetryVerbosity.HIGH);

    private final Arm pivotArm = new Arm(pivotArmConfig);

    public PivotSubsystem() {
        // Initialization handled fully by YAMS configurations above
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

    // ── Pivot Control ─────────────────────────────────────────────────────────

    /** Commands pivot to configured open position. */
    public void open() {
        pivotArm.setMechanismPositionSetpoint(Degrees.of(PIVOT_OPEN_ANGLE_DEG));
    }

    /** Commands pivot to configured closed position. */
    public void close() {
        pivotArm.setMechanismPositionSetpoint(Degrees.of(PIVOT_CLOSED_ANGLE_DEG));
    }

    /** * Command factory for moving to a specific target angle.
     * Use this in RobotContainer to easily bind buttons. 
     */
    public Command setAngleCommand(double degrees) {
        return pivotArm.run(Degrees.of(degrees));
    }

    /** Stops pivot output immediately. */
    public void stop() {
        pivotSmartController.stopClosedLoopController();
    }

    // ── Getters ───────────────────────────────────────────────────────────────

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