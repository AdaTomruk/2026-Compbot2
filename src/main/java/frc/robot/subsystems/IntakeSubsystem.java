package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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

    // ── Mechanism2d ───────────────────────────────────────────────────────────
    private final Mechanism2d intakeMech;
    private final MechanismLigament2d upperRollerViz;
    private final MechanismLigament2d bottomRollerViz;
    private final MechanismLigament2d outerRollerViz;

    // ── Visualization state ───────────────────────────────────────────────────
    private double upperAngleDeg = 0.0;
    private double bottomAngleDeg = 0.0;
    private double outerAngleDeg = 0.0;

    public IntakeSubsystem() {
        configureRoller(upperIndexMotor,   UPPER_INDEX_INVERTED);
        configureRoller(bottomRollerMotor, BOTTOM_ROLLER_INVERTED);
        configureRoller(outerRollerMotor,  OUTER_ROLLER_INVERTED);

        // 4x3 canvas for intake roller visualization
        intakeMech = new Mechanism2d(4, 3);

        MechanismRoot2d upperRoot = intakeMech.getRoot("UpperRoller", 1.0, 2.3);
        upperRollerViz = upperRoot.append(
            new MechanismLigament2d("Upper", 0.5, 0, 6, new Color8Bit(Color.kDodgerBlue))
        );

        MechanismRoot2d bottomRoot = intakeMech.getRoot("BottomRoller", 2.0, 1.5);
        bottomRollerViz = bottomRoot.append(
            new MechanismLigament2d("Bottom", 0.5, 0, 6, new Color8Bit(Color.kOrange))
        );

        MechanismRoot2d outerRoot = intakeMech.getRoot("OuterRoller", 3.0, 0.7);
        outerRollerViz = outerRoot.append(
            new MechanismLigament2d("Outer", 0.5, 0, 6, new Color8Bit(Color.kLimeGreen))
        );

        SmartDashboard.putData("Intake/Mechanism2d", intakeMech);
    }

    private void configureRoller(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(inverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ROLLER_SMART_CURRENT_LIMIT_A);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        double upperOutput = upperIndexMotor.get();
        double bottomOutput = bottomRollerMotor.get();
        double outerOutput = outerRollerMotor.get();

        // Show motor output magnitude as ligament length, clamped to [0, 1]
        upperRollerViz.setLength(Math.max(0.0, Math.min(Math.abs(upperOutput), 1.0)));
        bottomRollerViz.setLength(Math.max(0.0, Math.min(Math.abs(bottomOutput), 1.0)));
        outerRollerViz.setLength(Math.max(0.0, Math.min(Math.abs(outerOutput), 1.0)));

        // Animate roller direction and speed
        upperAngleDeg = (upperAngleDeg + upperOutput * 360.0 * 0.02) % 360.0;
        bottomAngleDeg = (bottomAngleDeg + bottomOutput * 360.0 * 0.02) % 360.0;
        outerAngleDeg = (outerAngleDeg + outerOutput * 360.0 * 0.02) % 360.0;

        upperRollerViz.setAngle(upperAngleDeg);
        bottomRollerViz.setAngle(bottomAngleDeg);
        outerRollerViz.setAngle(outerAngleDeg);

        SmartDashboard.putNumber("Intake/Upper_Output", upperOutput);
        SmartDashboard.putNumber("Intake/Bottom_Output", bottomOutput);
        SmartDashboard.putNumber("Intake/Outer_Output", outerOutput);
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