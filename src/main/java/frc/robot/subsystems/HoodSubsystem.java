package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
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
    private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

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
    private double sensorOffsetDeg = 0.0;
    private boolean hasSeeded = false;
    private Double pendingSeedAngleDeg = null;

    // --- Constructor ---
    public HoodSubsystem() {
        configureMotor();

        seedPositionDeg(HoodConstants.HOOD_MIN_ANGLE_DEG);

        if (RobotBase.isSimulation()) {
            configureSimulation();
        }

        SmartDashboard.putNumber("Hood Target Angle (deg)", targetAngleDeg);
    }

    // --- Motor Configuration ---
    private void configureMotor() {
        // SensorToMechanismRatio handles gear ratio internally
        // After this, getPosition() returns mechanism rotations automatically
        // NEVER multiply/divide by gear ratio manually anywhere else
        hoodConfig.Feedback.SensorToMechanismRatio = HoodConstants.HOOD_MOTOR_TO_MECHANISM_RATIO;

        // Current limits
    hoodConfig.CurrentLimits.SupplyCurrentLimit       = HoodConstants.HOOD_SUPPLY_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.StatorCurrentLimit       = HoodConstants.HOOD_STATOR_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // PID + Feedforward (Slot 0)
    hoodConfig.Slot0.kP = HoodConstants.HOOD_kP;
    hoodConfig.Slot0.kI = HoodConstants.HOOD_kI;
    hoodConfig.Slot0.kD = HoodConstants.HOOD_kD;
    hoodConfig.Slot0.kS = HoodConstants.HOOD_kS;
    hoodConfig.Slot0.kG = HoodConstants.HOOD_kG;
    hoodConfig.Slot0.kV = HoodConstants.HOOD_kV;

        // MotionMagic constraints (mechanism rotations/s and rotations/s²)
    hoodConfig.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.HOOD_MAX_VEL_RPS;
    hoodConfig.MotionMagic.MotionMagicAcceleration   = HoodConstants.HOOD_MAX_ACCEL_RPS2;

        // Soft limits in mechanism rotations (degrees / 360)
        // SensorToMechanismRatio already applied so just divide by 360
        updateSoftLimits();

        hoodConfig.MotorOutput.Inverted = HoodConstants.HOOD_INVERTED;

        hoodMotor.getConfigurator().apply(hoodConfig);
    }

    private void updateSoftLimits() {
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            HoodAngleUtil.toSoftLimitThreshold(HoodConstants.HOOD_MAX_ANGLE_DEG, sensorOffsetDeg);
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            HoodAngleUtil.toSoftLimitThreshold(HoodConstants.HOOD_MIN_ANGLE_DEG, sensorOffsetDeg);
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
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

    public void seedPositionDeg(double angleDeg) {
        pendingSeedAngleDeg = angleDeg;
        hasSeeded = false;
    }

    private void applyPendingSeed() {
        if (pendingSeedAngleDeg == null) {
            return;
        }

    var positionSignal = hoodMotor.getPosition();
        StatusCode status = BaseStatusSignal.refreshAll(positionSignal);
        if (!status.isOK()) {
            return;
        }
    double rawAngleDeg = positionSignal.getValueAsDouble() * 360.0;

    sensorOffsetDeg = HoodAngleUtil.computeSensorOffsetDeg(pendingSeedAngleDeg, rawAngleDeg);
        targetAngleDeg = pendingSeedAngleDeg;
        updateSoftLimits();
        hoodMotor.getConfigurator().apply(hoodConfig);

        hasSeeded = true;
        pendingSeedAngleDeg = null;

        if (pidEnabled) {
            applyMotionMagic();
        }

        System.out.println("Hood position seeded to: " + targetAngleDeg + "° (offset " + sensorOffsetDeg + "°)");
    }

    private void applyMotionMagic() {
        // Target in mechanism rotations = degrees / 360
        // SensorToMechanismRatio handles the rest internally
        hoodMotor.setControl(
            motionMagicRequest.withPosition(HoodAngleUtil.toRawRotations(targetAngleDeg, sensorOffsetDeg)));
    }

    // --- Getters ---
    public double getAngleDeg() {
        // getPosition() returns mechanism rotations → × 360 = real hood degrees
        // On boot: 18.2/360 * 360 = 18.2° ✓
        var positionSignal = hoodMotor.getPosition();
        BaseStatusSignal.refreshAll(positionSignal);
        return HoodAngleUtil.applyOffset(positionSignal.getValueAsDouble() * 360.0, sensorOffsetDeg);
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

        if (!hasSeeded && pendingSeedAngleDeg != null) {
            applyPendingSeed();
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
