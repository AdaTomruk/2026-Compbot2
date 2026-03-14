package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.config.FlywheelConfig;
import yams.mechanisms.rotational.Flywheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.ctre.TalonFXWrapper;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

/**
 * Flywheel shooter subsystem powered by YAMS.
 * Controls dual Kraken X60 motors for shooting game pieces.
 */
public class FlywheelSubsystem extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final TalonFX leftFlywheelMotor = new TalonFX(LEFT_FLYWHEEL_MOTOR_ID, CANBUS);
    private final TalonFX rightFlywheelMotor = new TalonFX(RIGHT_FLYWHEEL_MOTOR_ID, CANBUS);

    // ── YAMS Configuration ────────────────────────────────────────────────────
    private final SmartMotorControllerConfig leftFlywheelConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                FLYWHEEL_kP, FLYWHEEL_kI, FLYWHEEL_kD,
                RotationsPerSecond.of(STANDARD_SHOOT_RPM / 60.0),
                RotationsPerSecond.of(0.0)) // No acceleration constraint for flywheels
            .withFeedforward(new SimpleMotorFeedforward(FLYWHEEL_kS, FLYWHEEL_kV, FLYWHEEL_kA))
            .withTelemetry("LeftFlywheel", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(GearBox.fromRatio(FLYWHEEL_GEAR_RATIO)))
            .withMotorInverted(LEFT_FLYWHEEL_INVERTED)
            .withIdleMode(MotorMode.COAST)
            .withSupplyCurrentLimit(Amps.of(FLYWHEEL_SUPPLY_CURRENT_LIMIT))
            .withStatorCurrentLimit(Amps.of(FLYWHEEL_STATOR_CURRENT_LIMIT));

    private final SmartMotorController leftFlywheelController =
        new TalonFXWrapper(leftFlywheelMotor, DCMotor.getKrakenX60(1), leftFlywheelConfig);

    private final SmartMotorControllerConfig rightFlywheelConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                FLYWHEEL_kP, FLYWHEEL_kI, FLYWHEEL_kD,
                RotationsPerSecond.of(STANDARD_SHOOT_RPM / 60.0),
                RotationsPerSecond.of(0.0))
            .withFeedforward(new SimpleMotorFeedforward(FLYWHEEL_kS, FLYWHEEL_kV, FLYWHEEL_kA))
            .withTelemetry("RightFlywheel", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(GearBox.fromRatio(FLYWHEEL_GEAR_RATIO)))
            .withMotorInverted(RIGHT_FLYWHEEL_INVERTED)
            .withIdleMode(MotorMode.COAST)
            .withSupplyCurrentLimit(Amps.of(FLYWHEEL_SUPPLY_CURRENT_LIMIT))
            .withStatorCurrentLimit(Amps.of(FLYWHEEL_STATOR_CURRENT_LIMIT));

    private final SmartMotorController rightFlywheelController =
        new TalonFXWrapper(rightFlywheelMotor, DCMotor.getKrakenX60(1), rightFlywheelConfig);

    private final FlywheelConfig flywheelConfig =
        new FlywheelConfig(leftFlywheelController)
            .withFollower(rightFlywheelController)
            .withTelemetry("Flywheel", TelemetryVerbosity.HIGH);

    private final Flywheel flywheel = new Flywheel(flywheelConfig);

    public FlywheelSubsystem() {
        // Initialization handled fully by YAMS configurations above
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        flywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
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
        flywheel.setMechanismVelocitySetpoint(RotationsPerSecond.of(rpm / 60.0));
    }

    /** Stops the flywheel. */
    public void stop() {
        leftFlywheelController.stopClosedLoopController();
        rightFlywheelController.stopClosedLoopController();
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    /** @return Current flywheel speed in RPM (average of both motors). */
    public double getRPM() {
        double leftRPM = leftFlywheelController.getMechanismVelocity().in(RotationsPerSecond) * 60.0;
        double rightRPM = rightFlywheelController.getMechanismVelocity().in(RotationsPerSecond) * 60.0;
        return (leftRPM + rightRPM) / 2.0;
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
