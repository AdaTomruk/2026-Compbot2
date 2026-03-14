package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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

    // ── Mechanism2d ───────────────────────────────────────────────────────────
    private final Mechanism2d flywheelMech;
    private final MechanismLigament2d leftFlywheelArrow;
    private final MechanismLigament2d rightFlywheelArrow;
    private final MechanismLigament2d targetSpeedArrow;

    // ── State Tracking ────────────────────────────────────────────────────────
    private double targetRPM = 0.0;

    public FlywheelSubsystem() {
        // ── Motor Initialization ──────────────────────────────────────────────
        leftFlywheelMotor = new TalonFX(LEFT_FLYWHEEL_MOTOR_ID, CANBUS);
        rightFlywheelMotor = new TalonFX(RIGHT_FLYWHEEL_MOTOR_ID, CANBUS);

        configureMotors();

        // ── Mechanism2d Setup ─────────────────────────────────────────────────
        // 4x3 canvas for dual flywheel visualization
        flywheelMech = new Mechanism2d(4, 3);

        // Left flywheel (positioned at x=1, y=1.5)
        MechanismRoot2d leftRoot = flywheelMech.getRoot("LeftFlywheel", 1.0, 1.5);
        leftFlywheelArrow = leftRoot.append(
            new MechanismLigament2d("LeftSpeed", 0.0, 0, 8, new Color8Bit(Color.kOrange))
        );

        // Right flywheel (positioned at x=3, y=1.5)
        MechanismRoot2d rightRoot = flywheelMech.getRoot("RightFlywheel", 3.0, 1.5);
        rightFlywheelArrow = rightRoot.append(
            new MechanismLigament2d("RightSpeed", 0.0, 0, 8, new Color8Bit(Color.kOrangeRed))
        );

        // Target speed indicator (center, x=2, y=1.5)
        MechanismRoot2d targetRoot = flywheelMech.getRoot("Target", 2.0, 1.5);
        targetSpeedArrow = targetRoot.append(
            new MechanismLigament2d("TargetSpeed", 0.0, 90, 4, new Color8Bit(Color.kCyan))
        );

        SmartDashboard.putData("Flywheel/Mechanism2d", flywheelMech);
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
        // Get current speeds
        double leftRPS = leftFlywheelMotor.getVelocity().getValueAsDouble();
        double rightRPS = rightFlywheelMotor.getVelocity().getValueAsDouble();
        double leftRPM = leftRPS * 60.0;
        double rightRPM = rightRPS * 60.0;
        double avgRPM = (leftRPM + rightRPM) / 2.0;

        // Update Mechanism2d visualization
        // Scale speed to 0-1 length (max RPM = 6000 -> length 1.0)
        double maxRPMForDisplay = 6000.0;
        leftFlywheelArrow.setLength(Math.min(leftRPM / maxRPMForDisplay, 1.0));
        rightFlywheelArrow.setLength(Math.min(rightRPM / maxRPMForDisplay, 1.0));
        targetSpeedArrow.setLength(Math.min(targetRPM / maxRPMForDisplay, 1.0));

        // Rotate arrows to show spinning (visual effect)
        double leftAngle = (leftRPS * 360.0 * 0.02) % 360.0; // 20ms period
        double rightAngle = (rightRPS * 360.0 * 0.02) % 360.0;
        leftFlywheelArrow.setAngle(leftAngle);
        rightFlywheelArrow.setAngle(rightAngle);

        // SmartDashboard telemetry
        SmartDashboard.putNumber("Flywheel/Left_RPM", leftRPM);
        SmartDashboard.putNumber("Flywheel/Right_RPM", rightRPM);
        SmartDashboard.putNumber("Flywheel/Average_RPM", avgRPM);
        SmartDashboard.putNumber("Flywheel/Target_RPM", targetRPM);
        SmartDashboard.putNumber("Flywheel/RPM_Error", targetRPM - avgRPM);

        SmartDashboard.putNumber("Flywheel/Left_SupplyCurrent_A",
            leftFlywheelMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Right_SupplyCurrent_A",
            rightFlywheelMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Left_StatorCurrent_A",
            leftFlywheelMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Right_StatorCurrent_A",
            rightFlywheelMotor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putBoolean("Flywheel/At_Speed", atSpeed(targetRPM));
        SmartDashboard.putBoolean("Flywheel/At_Shoot_Speed", atShootSpeed());
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
        targetRPM = rpm;
        double rps = rpm / 60.0;
        leftFlywheelMotor.setControl(velocityRequest.withVelocity(rps));
        rightFlywheelMotor.setControl(velocityRequest.withVelocity(rps));
    }

    /** Stops the flywheel. */
    public void stop() {
        targetRPM = 0.0;
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
