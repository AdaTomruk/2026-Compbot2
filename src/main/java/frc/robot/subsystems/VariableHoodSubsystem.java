package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Variable hood subsystem.
 * Controls a single Kraken X60 motor to adjust the shooter's angle.
 */
public class VariableHoodSubsystem extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final TalonFX hoodMotor;

    // ── Control Requests ──────────────────────────────────────────────────────
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private final NeutralOut neutralRequest = new NeutralOut();

    public VariableHoodSubsystem() {
        hoodMotor = new TalonFX(HOOD_MOTOR_ID, CANBUS);
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor output
        config.MotorOutput.Inverted = HOOD_MOTOR_INVERTED ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Current limits
        config.CurrentLimits.SupplyCurrentLimit = HOOD_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = HOOD_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Gear ratio
        config.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;

        // PID & Feedforward (Slot 0)
        config.Slot0.kP = HOOD_kP;
        config.Slot0.kI = HOOD_kI;
        config.Slot0.kD = HOOD_kD;
        config.Slot0.kS = HOOD_kS;
        config.Slot0.kG = HOOD_kG;
        config.Slot0.kV = HOOD_kV;

        // Motion Magic
        // Convert from degrees/sec to rotations/sec for motor
        double cruiseVelocityRPS = (HOOD_MAX_VEL_DEG_PER_SEC / 360.0) * HOOD_GEAR_RATIO;
        double accelerationRPS2 = (HOOD_MAX_ACCEL_DEG_PER_SEC2 / 360.0) * HOOD_GEAR_RATIO;

        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocityRPS;
        config.MotionMagic.MotionMagicAcceleration = accelerationRPS2;
        config.MotionMagic.MotionMagicJerk = accelerationRPS2 * 10.0; // 10x acceleration

        // Soft limits (convert degrees to rotations)
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HOOD_MAX_ANGLE_DEG / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HOOD_MIN_ANGLE_DEG / 360.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        hoodMotor.getConfigurator().apply(config);

        // Set initial position to starting angle
        hoodMotor.setPosition(HOOD_STARTING_ANGLE_DEG / 360.0);
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Telemetry is handled by Phoenix signals
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
        double rotations = degrees / 360.0;
        hoodMotor.setControl(motionMagicRequest.withPosition(rotations));
    }

    /**
     * Command factory for moving to a specific target angle.
     * Use this in RobotContainer to easily bind buttons.
     */
    public Command setAngleCommand(double degrees) {
        return Commands.runOnce(() -> setAngle(degrees), this);
    }

    /** Stops hood movement immediately. */
    public void stop() {
        hoodMotor.setControl(neutralRequest);
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    /** @return Current hood angle in degrees. */
    public double getAngleDegrees() {
        return hoodMotor.getPosition().getValueAsDouble() * 360.0;
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
