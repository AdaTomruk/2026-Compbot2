package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

// ── NEW 2026 REVLib IMPORTS ───────────────────────────────────────────────
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake mechanism subsystem using three SPARK MAX motor controllers.
 */
public class IntakeSubsystem extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final SparkMax upperIndexMotor   = new SparkMax(UPPER_INDEX_MOTOR_ID,  MotorType.kBrushless);
    private final SparkMax bottomRollerMotor = new SparkMax(BOTTOM_ROLLER_MOTOR_ID, MotorType.kBrushless);
    private final TalonFX outerRollerMotor   = new TalonFX(OUTER_ROLLER_MOTOR_ID);

    // ── Control Requests ─────────────────────────────────────────────────────
    private final DutyCycleOut outerRollerRequest = new DutyCycleOut(0).withEnableFOC(false);
    private final NeutralOut outerRollerNeutralRequest = new NeutralOut();

    public IntakeSubsystem() {
        configureRoller(upperIndexMotor,   UPPER_INDEX_INVERTED);
        configureRoller(bottomRollerMotor, BOTTOM_ROLLER_INVERTED);
        configureOuterRoller();
    }

    private void configureRoller(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(inverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ROLLER_SMART_CURRENT_LIMIT_A);

        // Using the new 2026 root-level ResetMode and PersistMode
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void configureOuterRoller() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput
            .withInverted(OUTER_ROLLER_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

        config.CurrentLimits
            .withSupplyCurrentLimit(ROLLER_SMART_CURRENT_LIMIT_A)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(ROLLER_SMART_CURRENT_LIMIT_A)
            .withStatorCurrentLimitEnable(true);

        outerRollerMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        // SmartDashboard telemetry
        SmartDashboard.putNumber("Intake/Upper_Output", upperIndexMotor.get());
        SmartDashboard.putNumber("Intake/Bottom_Output", bottomRollerMotor.get());
        SmartDashboard.putNumber("Intake/Outer_Output",
            outerRollerMotor.getDutyCycle().getValueAsDouble());
    }

    // ── Roller control ────────────────────────────────────────────────────────

    /** Runs all intake rollers inward to acquire game pieces. */
    public void intake() {
        upperIndexMotor.set(UPPER_INDEX_INTAKE_OUTPUT);
        bottomRollerMotor.set(BOTTOM_ROLLER_INTAKE_OUTPUT);
    outerRollerMotor.setControl(outerRollerRequest.withOutput(OUTER_ROLLER_INTAKE_OUTPUT));
    }

    /** Runs all intake rollers outward to eject game pieces. */
    public void outtake() {
        upperIndexMotor.set(UPPER_INDEX_OUTTAKE_OUTPUT);
        bottomRollerMotor.set(BOTTOM_ROLLER_OUTTAKE_OUTPUT);
    outerRollerMotor.setControl(outerRollerRequest.withOutput(OUTER_ROLLER_OUTTAKE_OUTPUT));
    }

    /** Stops all roller motors. */
    public void stopRollers() {
        upperIndexMotor.stopMotor();
        bottomRollerMotor.stopMotor();
        outerRollerMotor.setControl(outerRollerNeutralRequest);
    }
}