package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Flywheel shooter subsystem.
 * Controls dual Kraken X60 motors for shooting game pieces.
 */
public class FlywheelSubsystem extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final TalonFX leftFlywheelMotor;
    private final TalonFX rightFlywheelMotor;

    // ── Control Requests ──────────────────────────────────────────────────────
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final NeutralOut neutralRequest = new NeutralOut();

    public FlywheelSubsystem() {
        // ── Motor Initialization ──────────────────────────────────────────────
        leftFlywheelMotor = new TalonFX(LEFT_FLYWHEEL_MOTOR_ID, CANBUS);
        rightFlywheelMotor = new TalonFX(RIGHT_FLYWHEEL_MOTOR_ID, CANBUS);

        configureMotors();
    }

    private void configureMotors() {
        // ── Left Motor Configuration ──────────────────────────────────────────
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();

        // Motor output
        leftConfig.MotorOutput.Inverted = LEFT_FLYWHEEL_INVERTED ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current limits
        leftConfig.CurrentLimits.SupplyCurrentLimit = FLYWHEEL_SUPPLY_CURRENT_LIMIT;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftConfig.CurrentLimits.StatorCurrentLimit = FLYWHEEL_STATOR_CURRENT_LIMIT;
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // PID & Feedforward (Slot 0)
        leftConfig.Slot0.kP = FLYWHEEL_kP;
        leftConfig.Slot0.kI = FLYWHEEL_kI;
        leftConfig.Slot0.kD = FLYWHEEL_kD;
        leftConfig.Slot0.kS = FLYWHEEL_kS;
        leftConfig.Slot0.kV = FLYWHEEL_kV;
        leftConfig.Slot0.kA = FLYWHEEL_kA;

        // Gear ratio (1:1 direct drive)
        leftConfig.Feedback.SensorToMechanismRatio = FLYWHEEL_GEAR_RATIO;

        leftFlywheelMotor.getConfigurator().apply(leftConfig);

        // ── Right Motor Configuration (Follower) ──────────────────────────────
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();

        rightConfig.MotorOutput.Inverted = RIGHT_FLYWHEEL_INVERTED ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rightConfig.CurrentLimits.SupplyCurrentLimit = FLYWHEEL_SUPPLY_CURRENT_LIMIT;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightConfig.CurrentLimits.StatorCurrentLimit = FLYWHEEL_STATOR_CURRENT_LIMIT;
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        rightConfig.Slot0.kP = FLYWHEEL_kP;
        rightConfig.Slot0.kI = FLYWHEEL_kI;
        rightConfig.Slot0.kD = FLYWHEEL_kD;
        rightConfig.Slot0.kS = FLYWHEEL_kS;
        rightConfig.Slot0.kV = FLYWHEEL_kV;
        rightConfig.Slot0.kA = FLYWHEEL_kA;

        rightConfig.Feedback.SensorToMechanismRatio = FLYWHEEL_GEAR_RATIO;

        rightFlywheelMotor.getConfigurator().apply(rightConfig);
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Telemetry is handled by Phoenix signals
    }

    // ── Flywheel Control ──────────────────────────────────────────────────────

    /** Sets flywheel to standard shooting speed. */
    public void shoot() {
        setRPM(STANDARD_SHOOT_RPM);
    }

    /** Sets flywheel to warm-up speed (75% of shooting speed). */
    public void warmUp() {
        setRPM(WARM_UP_RPM);
    }

    /** Sets flywheel to a specific RPM. */
    public void setRPM(double rpm) {
        double rps = rpm / 60.0;
        leftFlywheelMotor.setControl(velocityRequest.withVelocity(rps));
        rightFlywheelMotor.setControl(velocityRequest.withVelocity(rps));
    }

    /** Stops the flywheel. */
    public void stop() {
        leftFlywheelMotor.setControl(neutralRequest);
        rightFlywheelMotor.setControl(neutralRequest);
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    /** @return Current flywheel speed in RPM (average of both motors). */
    public double getRPM() {
        double leftRPS = leftFlywheelMotor.getVelocity().getValueAsDouble();
        double rightRPS = rightFlywheelMotor.getVelocity().getValueAsDouble();
        return ((leftRPS + rightRPS) / 2.0) * 60.0;
    }

    /** @return true when flywheel is at target speed within tolerance. */
    public boolean atSpeed(double targetRPM) {
        return Math.abs(getRPM() - targetRPM) <= FLYWHEEL_AT_SPEED_TOLERANCE_RPM;
    }

    /** @return true when flywheel is at standard shooting speed. */
    public boolean atShootSpeed() {
        return atSpeed(STANDARD_SHOOT_RPM);
    }
}
