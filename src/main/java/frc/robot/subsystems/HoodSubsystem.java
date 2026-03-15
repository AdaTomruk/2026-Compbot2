package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {

    // --- Motor ---
    private final TalonFX hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID);

    // --- Control Requests ---
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private final NeutralOut neutralRequest = new NeutralOut();

    // --- Simulation ---
    private TalonFXSimState hoodMotorSim;
    private SingleJointedArmSim hoodArmSim;

    // --- State ---
    // Motor boots at 0 = hood physically at HOOD_MIN_ANGLE_DEG (18.2°)
    // All angles throughout this class are real hood degrees (18.2° to 56.2°)
    private boolean pidEnabled    = false;
    private double targetAngleDeg = HoodConstants.HOOD_MIN_ANGLE_DEG;

    // --- Constructor ---
    public HoodSubsystem() {
        configureMotor();

        if (RobotBase.isSimulation()) {
            configureSimulation();
        }

        SmartDashboard.putNumber("Hood Target Angle (deg)", targetAngleDeg);
    }

    // --- Motor Configuration ---
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // SensorToMechanismRatio handles gear ratio internally
        // After this, getPosition() returns mechanism rotations automatically
        // NEVER multiply/divide by gear ratio manually anywhere else
        config.Feedback.SensorToMechanismRatio = HoodConstants.HOOD_MOTOR_TO_MECHANISM_RATIO;

        // Current limits
        config.CurrentLimits.SupplyCurrentLimit       = HoodConstants.HOOD_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit       = HoodConstants.HOOD_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // PID + Feedforward (Slot 0)
        config.Slot0.kP = HoodConstants.HOOD_kP;
        config.Slot0.kI = HoodConstants.HOOD_kI;
        config.Slot0.kD = HoodConstants.HOOD_kD;
        config.Slot0.kS = HoodConstants.HOOD_kS;
        config.Slot0.kG = HoodConstants.HOOD_kG;
        config.Slot0.kV = HoodConstants.HOOD_kV;

        // MotionMagic constraints (mechanism rotations/s and rotations/s²)
        config.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.HOOD_MAX_VEL_RPS;
        config.MotionMagic.MotionMagicAcceleration   = HoodConstants.HOOD_MAX_ACCEL_RPS2;

        // Soft limits in mechanism rotations (degrees / 360)
        // SensorToMechanismRatio already applied so just divide by 360
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.HOOD_MAX_ANGLE_DEG / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.HOOD_MIN_ANGLE_DEG / 360.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;

        config.MotorOutput.Inverted = HoodConstants.HOOD_INVERTED;

        hoodMotor.getConfigurator().apply(config);

        // Motor boots at 0 but hood is physically at 18.2°
        // Seed so getPosition() returns 18.2/360 mechanism rotations
        // SensorToMechanismRatio handles gear ratio — only divide by 360 here
        hoodMotor.setPosition(HoodConstants.HOOD_MIN_ANGLE_DEG / 360.0);
    }

    // --- Simulation Configuration ---
    private void configureSimulation() {
        hoodMotorSim = hoodMotor.getSimState();

        hoodArmSim = new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            HoodConstants.HOOD_MOTOR_TO_MECHANISM_RATIO,
            SingleJointedArmSim.estimateMOI(
                HoodConstants.HOOD_ARM_LENGTH_METERS,
                HoodConstants.HOOD_ARM_MASS_KG),
            HoodConstants.HOOD_ARM_LENGTH_METERS,
            Math.toRadians(HoodConstants.HOOD_MIN_ANGLE_DEG),
            Math.toRadians(HoodConstants.HOOD_MAX_ANGLE_DEG),
            false, // disable gravity until kG is tuned
            Math.toRadians(HoodConstants.HOOD_MIN_ANGLE_DEG));
    }

    // --- Simulation Update ---
    private void updateSimulation() {
        hoodMotorSim.setSupplyVoltage(12.0);

        // Feed motor voltage into physics sim
        hoodArmSim.setInputVoltage(hoodMotorSim.getMotorVoltage());

        // Step physics 20ms forward
        hoodArmSim.update(0.020);

        // arm rad/s → motor RPS (× gear ratio / 2π)
        double motorVelocityRps = hoodArmSim.getVelocityRadPerSec()
            * HoodConstants.HOOD_MOTOR_TO_MECHANISM_RATIO
            / (2.0 * Math.PI);

        // arm radians → motor rotations (× gear ratio / 2π)
        double motorPositionRot = (hoodArmSim.getAngleRads() / (2.0 * Math.PI))
            * HoodConstants.HOOD_MOTOR_TO_MECHANISM_RATIO;

        // Push back into TalonFX sim — always raw rotor values here
        hoodMotorSim.setRawRotorPosition(motorPositionRot);
        hoodMotorSim.setRotorVelocity(motorVelocityRps);
    }

    // --- Control Methods ---
    public void setAngle(double degrees) {
        // Clamp to physical limits
        targetAngleDeg = Math.max(HoodConstants.HOOD_MIN_ANGLE_DEG,
                         Math.min(HoodConstants.HOOD_MAX_ANGLE_DEG, degrees));

        if (pidEnabled) {
            applyMotionMagic();
        }

        System.out.println("Hood target set to: " + targetAngleDeg + "°");
    }

    public void enablePID() {
        pidEnabled = true;
        applyMotionMagic();
        System.out.println("Hood PID Enabled — target: " + targetAngleDeg + "°");
    }

    public void disablePID() {
        pidEnabled = false;
        hoodMotor.setControl(neutralRequest);
        System.out.println("Hood PID Disabled");
    }

    private void applyMotionMagic() {
        // Target in mechanism rotations = degrees / 360
        // SensorToMechanismRatio handles the rest internally
        hoodMotor.setControl(
            motionMagicRequest.withPosition(targetAngleDeg / 360.0));
    }

    // --- Getters ---
    public double getAngleDeg() {
        // getPosition() returns mechanism rotations → × 360 = real hood degrees
        // On boot: 18.2/360 * 360 = 18.2° ✓
        return hoodMotor.getPosition().getValueAsDouble() * 360.0;
    }

    public boolean atTarget() {
        return Math.abs(getAngleDeg() - targetAngleDeg) < HoodConstants.HOOD_AT_TARGET_TOLERANCE_DEG;
    }

    public boolean isPIDEnabled() {
        return pidEnabled;
    }

    // --- Periodic ---
    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            updateSimulation();
        }

        // Read SmartDashboard target — only apply if changed
        double dashTarget = SmartDashboard.getNumber("Hood Target Angle (deg)", targetAngleDeg);
        if (dashTarget != targetAngleDeg) {
            setAngle(dashTarget);
        }

        SmartDashboard.putNumber("Hood Angle (deg)", getAngleDeg());
        SmartDashboard.putNumber("Hood Target (deg)", targetAngleDeg);
        SmartDashboard.putBoolean("Hood PID Enabled", pidEnabled);
        SmartDashboard.putBoolean("Hood At Target", atTarget());
    }
}
