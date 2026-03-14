package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake mechanism subsystem using three SPARK MAX motor controllers:
 * <ul>
 * <li>31: NEO Vortex upper indexing roller</li>
 * <li>32: NEO Vortex lower roller/mecanum stage</li>
 * <li>34: NEO 1.1 outer roller</li>
 * </ul>
 */
public class IntakeSubsystem extends SubsystemBase {

    // ── Roller motors ─────────────────────────────────────────────────────────
    private final SparkMax upperIndexMotor   = new SparkMax(UPPER_INDEX_MOTOR_ID,  MotorType.kBrushless);
    private final SparkMax bottomRollerMotor = new SparkMax(BOTTOM_ROLLER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax outerRollerMotor  = new SparkMax(OUTER_ROLLER_MOTOR_ID,  MotorType.kBrushless);

    public IntakeSubsystem() {
        configureRoller(upperIndexMotor,   UPPER_INDEX_INVERTED);
        configureRoller(bottomRollerMotor, BOTTOM_ROLLER_INVERTED);
        configureRoller(outerRollerMotor,  OUTER_ROLLER_INVERTED);
    }

    private void configureRoller(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(inverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ROLLER_SMART_CURRENT_LIMIT_A);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
}