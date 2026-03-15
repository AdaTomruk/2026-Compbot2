package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeArmSubsystem extends SubsystemBase {

    // --- Motor, Encoder, Controller ---
    private final SparkMax pivotMotor = new SparkMax(IntakeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    private RelativeEncoder pivotEncoder;
    private SparkClosedLoopController pivotPIDController;

    // --- Feedforward ---
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(
        IntakeConstants.PIVOT_kS,
        IntakeConstants.PIVOT_kG,
        IntakeConstants.PIVOT_kV);

    // --- Motion Profile ---
    private final TrapezoidProfile pivotTrapezoidProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            IntakeConstants.PIVOT_MAX_VEL_DEG_PER_SEC,
            IntakeConstants.PIVOT_MAX_ACCEL_DEG_PER_SEC2));

    private TrapezoidProfile.State pivotCurState = new TrapezoidProfile.State();
    private TrapezoidProfile.State pivotGoalState = new TrapezoidProfile.State();
    private double prevUpdateTime = Timer.getFPGATimestamp();

    // --- State ---
    public static boolean isOpen = false;

    // --- Periodic IO ---
    private final PeriodicIO mPeriodicIO = new PeriodicIO();

    private static class PeriodicIO {
        double arm_target = IntakeConstants.PIVOT_CLOSED_ANGLE_DEG;
        ArmState state = ArmState.CLOSE;
    }

    public enum ArmState {
        OPEN,
        CLOSE
    }

    // --- Constructor ---
    public IntakeArmSubsystem() {
        configureMotor();

        SmartDashboard.putNumber("Pivot Open Angle (deg)", IntakeConstants.PIVOT_OPEN_ANGLE_DEG);
        SmartDashboard.putNumber("Pivot Closed Angle (deg)", IntakeConstants.PIVOT_CLOSED_ANGLE_DEG);
    }

    // --- Configuration ---
    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        // Degrees per motor rotation = 360 / gear ratio
        double positionConversionFactor = 360.0 / IntakeConstants.PIVOT_GEAR_RATIO;

        config.inverted(IntakeConstants.PIVOT_INVERTED);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(IntakeConstants.PIVOT_SMART_CURRENT_LIMIT_A);

        config.encoder
            .positionConversionFactor(positionConversionFactor);   // motor rotations → degrees

        config.closedLoop
            .pid(IntakeConstants.PIVOT_kP, IntakeConstants.PIVOT_kI, IntakeConstants.PIVOT_kD);

        config.softLimit
            .forwardSoftLimit(IntakeConstants.PIVOT_SOFT_MAX_ANGLE_DEG)
            .reverseSoftLimit(IntakeConstants.PIVOT_SOFT_MIN_ANGLE_DEG)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        pivotEncoder = pivotMotor.getEncoder();
        pivotPIDController = pivotMotor.getClosedLoopController();

        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Seed motion profile current state from actual encoder position
        pivotCurState.position = pivotEncoder.getPosition();
        pivotCurState.velocity = 0.0;
    }

    // --- Control Methods ---
    private void openArm() {
        isOpen = true;
        mPeriodicIO.state = ArmState.OPEN;
        mPeriodicIO.arm_target = IntakeConstants.PIVOT_OPEN_ANGLE_DEG;
    }

    private void closeArm() {
        isOpen = false;
        mPeriodicIO.state = ArmState.CLOSE;
        mPeriodicIO.arm_target = IntakeConstants.PIVOT_CLOSED_ANGLE_DEG;
    }

    public void toggleArm() {
        if (isOpen) {
            closeArm();
            System.out.println("Closing Intake Arm");
        } else {
            openArm();
            System.out.println("Opening Intake Arm");
        }
    }

    // --- Getters ---
    public double getAngleDeg() {
        return pivotEncoder.getPosition();
    }

    public boolean atTarget() {
        return Math.abs(getAngleDeg() - mPeriodicIO.arm_target) < IntakeConstants.PIVOT_AT_TARGET_TOLERANCE_DEG;
    }

    public ArmState getState() {
        return mPeriodicIO.state;
    }

    // --- Write Outputs ---
    private void writePeriodicOutputs() {
        double curTime = Timer.getFPGATimestamp();
        double dt = curTime - prevUpdateTime;
        prevUpdateTime = curTime;

        pivotGoalState.position = mPeriodicIO.arm_target;
        pivotCurState = pivotTrapezoidProfile.calculate(dt, pivotCurState, pivotGoalState);

        // Convert degrees to radians for ArmFeedforward (expects radians)
        double posRad = Math.toRadians(pivotCurState.position);
        double velRad = Math.toRadians(pivotCurState.velocity);
        double arbFF = pivotFeedforward.calculate(posRad, velRad);

        pivotPIDController.setReference(
            pivotCurState.position,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            arbFF,
            ArbFFUnits.kVoltage);
    }

    // --- Periodic ---
    @Override
    public void periodic() {
        writePeriodicOutputs();

        SmartDashboard.putBoolean("Intake Arm Open", isOpen);
        SmartDashboard.putBoolean("Intake Arm At Target", atTarget());
        SmartDashboard.putNumber("Intake Arm Angle (deg)", getAngleDeg());
        SmartDashboard.putString("Intake Arm State", mPeriodicIO.state.toString());
        SmartDashboard.putNumber("Intake Arm Target (deg)", mPeriodicIO.arm_target);
    }
}