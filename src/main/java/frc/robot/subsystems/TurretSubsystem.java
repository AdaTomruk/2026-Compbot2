package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretSubsystem extends SubsystemBase {

    // ── Hardware ───────────────────────────────────────────────────────────────
    private final TalonFX  turretMotor;
    private final CANcoder turretEncoder10t;
    private final CANcoder turretEncoder17t;

    // ── Status Signals ─────────────────────────────────────────────────────────
    private final StatusSignal<Angle> cancoder10tPositionSignal;
    private final StatusSignal<Angle> cancoder17tPositionSignal;

    // ── CRT ────────────────────────────────────────────────────────────────────
    private final EasyCRTConfig easyCrtConfig;
    private final EasyCRT       easyCrtSolver;

    // tracks whether CRT has successfully seeded the motor encoder
    private boolean crtSeeded = false;

    // ── Chassis Heading Supplier ───────────────────────────────────────────────
    private final Supplier<Rotation2d> chassisHeadingSupplier;

    // ── Control Requests ───────────────────────────────────────────────────────
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private final NeutralOut         neutralRequest      = new NeutralOut();

    // ── State ──────────────────────────────────────────────────────────────────
    private double  fieldTargetDegrees         = 0.0;
    private boolean chassisCompensationEnabled = false;

    // ── Mechanism2d ───────────────────────────────────────────────────────────
    private final Mechanism2d         turretMech;
    private final MechanismLigament2d turretArrow;
    private final MechanismLigament2d turretTargetArrow;

    // ── Simulation ────────────────────────────────────────────────────────────
    private DCMotorSim turretMotorSim;

    /**
     * @param chassisHeadingSupplier supplier of the robot's current field-relative heading
     *                               Pass: () -> drivetrain.getState().Pose.getRotation()
     */
    public TurretSubsystem(Supplier<Rotation2d> chassisHeadingSupplier) {
        this.chassisHeadingSupplier = chassisHeadingSupplier;

        // ── Motor ──────────────────────────────────────────────────────────────
        turretMotor = new TalonFX(TurretConstants.TURN_MOTOR_ID, TurretConstants.CANBUS);
        configureTurretMotor();

        // ── Encoders ───────────────────────────────────────────────────────────
        turretEncoder10t = new CANcoder(TurretConstants.ENCODER_10T_ID, TurretConstants.CANBUS);
        turretEncoder17t = new CANcoder(TurretConstants.ENCODER_17T_ID, TurretConstants.CANBUS);

        cancoder10tPositionSignal = turretEncoder10t.getAbsolutePosition();
        cancoder17tPositionSignal = turretEncoder17t.getAbsolutePosition();

        cancoder10tPositionSignal.setUpdateFrequency(50);
        cancoder17tPositionSignal.setUpdateFrequency(50);

        // ── EasyCRT ────────────────────────────────────────────────────────────
        Supplier<Angle> enc10tSupplier = () -> {
            cancoder10tPositionSignal.refresh();
            return cancoder10tPositionSignal.getValue();
        };
        Supplier<Angle> enc17tSupplier = () -> {
            cancoder17tPositionSignal.refresh();
            return cancoder17tPositionSignal.getValue();
        };

        easyCrtConfig =
            new EasyCRTConfig(enc10tSupplier, enc17tSupplier)
                .withCommonDriveGear(
                    /* commonRatio                */ 1.0,
                    /* driveGearTeeth (85T ring)  */ TurretConstants.TURRET_RING_GEAR_TEETH,
                    /* encoder1 pinion (10T)      */ TurretConstants.ENCODER_10T_PINION_TEETH,
                    /* encoder2 pinion (17T)      */ TurretConstants.ENCODER_17T_PINION_TEETH)
                .withAbsoluteEncoderOffsets(
                    Rotations.of(TurretConstants.ENCODER_10T_OFFSET_ROT),
                    Rotations.of(TurretConstants.ENCODER_17T_OFFSET_ROT))
                .withMechanismRange(
                    Rotations.of(TurretConstants.MIN_ANGLE_ROT),
                    Rotations.of(TurretConstants.MAX_ANGLE_ROT))
                .withMatchTolerance(Rotations.of(TurretConstants.CRT_MATCH_TOLERANCE_ROT))
                .withAbsoluteEncoderInversions(false, false);

        easyCrtSolver = new EasyCRT(easyCrtConfig);

        // ── Mechanism2d ────────────────────────────────────────────────────────
        turretMech = new Mechanism2d(3, 3);
        MechanismRoot2d root = turretMech.getRoot("Turret", 1.5, 1.5);

        // Grey line — always points up, represents robot forward
        root.append(new MechanismLigament2d(
            "RobotFront",
            0.6,
            90,
            2,
            new Color8Bit(100, 100, 100)
        ));

        // Orange — actual turret heading
        turretArrow = root.append(new MechanismLigament2d(
            "TurretHeading",
            1.0,
            90,
            6,
            new Color8Bit(255, 165, 0)
        ));

        // Cyan — target heading MotionMagic is driving to
        turretTargetArrow = root.append(new MechanismLigament2d(
            "TurretTarget",
            0.7,
            90,
            3,
            new Color8Bit(0, 200, 255)
        ));

        SmartDashboard.putData("Turret/Mechanism2d", turretMech);

        // ── Simulation ─────────────────────────────────────────────────────────
        if (RobotBase.isSimulation()) {
            turretMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(1),
                    0.001,
                    TurretConstants.MOTOR_TO_MECHANISM_RATIO
                ),
                DCMotor.getKrakenX60Foc(1)
            );

            // ADD THESE TWO LINES TO "POWER ON" THE SENSORS IN SIMULATION
            turretEncoder10t.getSimState().setSupplyVoltage(12.0);
            turretEncoder17t.getSimState().setSupplyVoltage(12.0);
            
            // In sim, CRT will not work — mark as seeded at 0 so robot can be tested
            turretMotor.setPosition(0);
            crtSeeded = true;
        }
    }

    // ── Motor Configuration ────────────────────────────────────────────────────

    private void configureTurretMotor() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.Inverted    = TurretConstants.MOTOR_INVERTED
                                        ? InvertedValue.Clockwise_Positive
                                        : InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = TurretConstants.SUPPLY_CURRENT_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = TurretConstants.STATOR_CURRENT_LIMIT;

        cfg.MotorOutput.PeakForwardDutyCycle =  TurretConstants.MAX_OUTPUT;
        cfg.MotorOutput.PeakReverseDutyCycle = -TurretConstants.MAX_OUTPUT;

        cfg.Slot0.kP = TurretConstants.kP;
        cfg.Slot0.kI = TurretConstants.kI;
        cfg.Slot0.kD = TurretConstants.kD;
        cfg.Slot0.kS = TurretConstants.kS;
        cfg.Slot0.kV = TurretConstants.kV;
        cfg.Slot0.kA = TurretConstants.kA;

        cfg.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MM_CRUISE_VELOCITY;
        cfg.MotionMagic.MotionMagicAcceleration   = TurretConstants.MM_ACCELERATION;
        cfg.MotionMagic.MotionMagicJerk           = TurretConstants.MM_JERK;

        // Soft limits in motor rotations
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            TurretConstants.MAX_ANGLE_ROT * TurretConstants.MOTOR_TO_MECHANISM_RATIO;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            TurretConstants.MIN_ANGLE_ROT * TurretConstants.MOTOR_TO_MECHANISM_RATIO;

        turretMotor.getConfigurator().apply(cfg);
        turretMotor.optimizeBusUtilization();
    }

    // ── CRT Seeding ───────────────────────────────────────────────────────────

    /**
     * Seeds the TalonFX internal encoder with the CRT-resolved absolute angle.
     * Must be called at teleopInit and autonomousInit — NOT in the constructor,
     * because CAN signals need time to stabilize after boot.
     *
     * Safe to call multiple times — re-seeds on every enable, correcting any
     * encoder drift that occurred while disabled.
     */
    public void seedMotorFromCRT() {
        // In simulation CRT doesn't work — skip silently
        if (RobotBase.isSimulation()) return;

        getCRTAngle().ifPresentOrElse(
            mechAngle -> {
                double mechRotations = mechAngle.in(Rotations);
                double motorRotations = mechRotations * TurretConstants.MOTOR_TO_MECHANISM_RATIO;

                turretMotor.setPosition(motorRotations);
                crtSeeded = true;

                System.out.printf("[CRT] Seeded at %.3f rot (%.1f deg)%n",
                    mechRotations, mechRotations * 360.0);

                SmartDashboard.putBoolean("Turret/CRT_Seeded",       true);
                SmartDashboard.putNumber ("Turret/CRT_Seed_Degrees",  mechRotations * 360.0);
            },
            () -> {
                crtSeeded = false;
                System.out.println("[CRT] WARNING — seed FAILED. Check CANcoder offsets and wiring.");
                System.out.println("[CRT] Do NOT run turret until seed succeeds.");
                SmartDashboard.putBoolean("Turret/CRT_Seeded", false);
            }
        );
    }

    /**
     * Returns true if CRT has successfully seeded the motor encoder at least once
     * this enable cycle. Use this as a guard before allowing turret commands.
     */
    public boolean isCRTSeeded() {
        return crtSeeded;
    }

    // ── Core Private Movement ──────────────────────────────────────────────────

    /**
     * All movement goes through here.
     * Guards against running before CRT has seeded — logs a warning instead of moving.
     */
    private void driveToRotations(double rawTargetRotations) {
        // Block movement if CRT hasn't seeded yet — position is unknown
        if (!crtSeeded) {
            System.out.println("[Turret] WARNING — blocked move: CRT not seeded yet");
            SmartDashboard.putBoolean("Turret/Blocked_No_Seed", true);
            return;
        }
        SmartDashboard.putBoolean("Turret/Blocked_No_Seed", false);

        double currentRotations = getMechanismRotations();

        // 1. Find the shortest path delta [-0.5, 0.5] rotations
        double delta = rawTargetRotations - currentRotations;
        delta = delta - Math.round(delta); 
        
        // 2. Calculate the position if we take the shortest path
        double primaryTarget = currentRotations + delta;
        
        // 3. Calculate the position if we take the "long way" (add/subtract 1 full rotation)
        // If delta is positive, the alternate is negative (subtract 1). 
        // If delta is negative, the alternate is positive (add 1).
        double alternateTarget = primaryTarget + (delta > 0 ? -1.0 : 1.0);

        // 4. Check which targets are within our physical/software constraints
        boolean primaryValid = primaryTarget >= TurretConstants.MIN_ANGLE_ROT 
                            && primaryTarget <= TurretConstants.MAX_ANGLE_ROT;
                            
        boolean alternateValid = alternateTarget >= TurretConstants.MIN_ANGLE_ROT 
                              && alternateTarget <= TurretConstants.MAX_ANGLE_ROT;

        double finalTarget;

        // 5. Route to the best target
        if (primaryValid) {
            // Shortest path is safe — do that
            finalTarget = primaryTarget;
        } else if (alternateValid) {
            // Shortest path hits a limit, but the long way is safe! (Triggers the unwind)
            finalTarget = alternateTarget;
        } else {
            // Neither is safe (e.g., target is physically impossible). 
            // Clamp the primary target to the nearest limit so it gets as close as possible.
            finalTarget = Math.max(
                TurretConstants.MIN_ANGLE_ROT,
                Math.min(TurretConstants.MAX_ANGLE_ROT, primaryTarget)
            );
        }

        turretMotor.setControl(
            motionMagicRequest.withPosition(
                finalTarget * TurretConstants.MOTOR_TO_MECHANISM_RATIO));
    }

    // ── Public Control API ─────────────────────────────────────────────────────

    /**
     * Sets an absolute robot-relative heading. Disables chassis compensation.
     * @param degrees 0° = robot forward
     */
    public void setHeading(double degrees) {
        chassisCompensationEnabled = false;
        driveToRotations(degrees / 360.0);
    }

    /**
     * Rotates by a relative offset from current position. Disables chassis compensation.
     * @param degrees positive or negative degrees to add
     */
    public void rotateBy(double degrees) {
        chassisCompensationEnabled = false;
        driveToRotations((getMechanismDegrees() + degrees) / 360.0);
    }

    /**
     * Locks the turret to a field-relative heading, compensating for chassis rotation.
     * Call once — periodic() maintains the lock every 20ms automatically.
     * @param fieldDegrees field-relative angle to hold
     */
    public void setFieldHeading(double fieldDegrees) {
        chassisCompensationEnabled = true;
        fieldTargetDegrees         = fieldDegrees;
        applyChassisCompensation();
    }

    /** Re-enables chassis compensation using the last stored field heading. */
    public void enableChassisCompensation() {
        chassisCompensationEnabled = true;
    }

    /** Disables chassis compensation. Turret holds its current robot-relative angle. */
    public void disableChassisCompensation() {
        chassisCompensationEnabled = false;
    }

    /** Stops the turret. Disables chassis compensation. */
    public void stop() {
        chassisCompensationEnabled = false;
        turretMotor.setControl(neutralRequest);
    }

    /** Open loop output — direction test only (Step 5). Remove after testing. */
    public void testOpenLoop(double percentOutput) {
        turretMotor.set(percentOutput);
    }

    // ── Chassis Compensation ───────────────────────────────────────────────────

    private void applyChassisCompensation() {
        double chassisDegrees = chassisHeadingSupplier.get().getDegrees();
        double robotRelative  = fieldTargetDegrees - chassisDegrees;
        driveToRotations(robotRelative / 360.0);
    }

    // ── Getters ────────────────────────────────────────────────────────────────

    /** CRT-resolved absolute mechanism angle — only reliable when stationary. */
    public Optional<Angle> getCRTAngle() {
        return easyCrtSolver.getAngleOptional();
    }

    public double getCRTAngleRotations() {
        return getCRTAngle().map(a -> a.in(Rotations)).orElse(Double.NaN);
    }

    public double getCRTAngleDegrees() {
        return getCRTAngleRotations() * 360.0;
    }

    /** Current mechanism position in rotations from the TalonFX internal encoder. */
    public double getMechanismRotations() {
        return turretMotor.getPosition().getValueAsDouble()
               / TurretConstants.MOTOR_TO_MECHANISM_RATIO;
    }

    /** Current mechanism position in degrees from the TalonFX internal encoder. */
    public double getMechanismDegrees() {
        return getMechanismRotations() * 360.0;
    }

    /** True when within AT_TARGET_TOLERANCE_DEG of the commanded target. */
    public boolean atTarget() {
        return Math.abs(turretMotor.getClosedLoopError().getValueAsDouble())
               < (TurretConstants.AT_TARGET_TOLERANCE_DEG / 360.0
                  * TurretConstants.MOTOR_TO_MECHANISM_RATIO);
    }

    public boolean isChassisCompensationEnabled() {
        return chassisCompensationEnabled;
    }

    // ── Periodic ───────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // 1. ── Simulation (Run this FIRST so the rest of the loop has fresh data) ──
        if (RobotBase.isSimulation()) {
            var simState = turretMotor.getSimState();
            simState.setSupplyVoltage(12.0);
            turretMotorSim.setInputVoltage(simState.getMotorVoltage());
            turretMotorSim.update(0.02);

            // FIX: Multiply WPILib mechanism output by gear ratio to get CTRE rotor position
            simState.setRawRotorPosition(
                turretMotorSim.getAngularPositionRotations() * TurretConstants.MOTOR_TO_MECHANISM_RATIO
            );
            simState.setRotorVelocity(
                (turretMotorSim.getAngularVelocityRPM() / 60.0) * TurretConstants.MOTOR_TO_MECHANISM_RATIO
            );

            // FIX: DCMotorSim outputs mechanism position, so no need to divide by gear ratio here anymore
            SmartDashboard.putNumber("Turret/Sim_Mechanism_Degrees",
                turretMotorSim.getAngularPositionRotations() * 360.0);
        }

        // 2. ── Chassis compensation — re-applies every 20ms ───────────────────────
        if (chassisCompensationEnabled) {
            applyChassisCompensation();
        }

        // 3. ── Mechanism2d (Now gets the freshest data) ───────────────────────────
        double currentDeg = getMechanismDegrees();
        double targetDeg  = turretMotor.getClosedLoopReference().getValueAsDouble()
                            / TurretConstants.MOTOR_TO_MECHANISM_RATIO * 360.0;

        turretArrow.setAngle(90 - currentDeg);      // orange — actual
        turretTargetArrow.setAngle(90 - targetDeg); // cyan   — target

        // 4. ── Telemetry ──────────────────────────────────────────────────────────
        SmartDashboard.putNumber ("Turret/Mechanism_Degrees",         currentDeg);
        SmartDashboard.putNumber ("Turret/Mechanism_Rotations",       getMechanismRotations());
        SmartDashboard.putNumber ("Turret/Target_Degrees",            targetDeg);
        SmartDashboard.putNumber ("Turret/Motor_Rotations",           turretMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber ("Turret/Velocity_RPS",              turretMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber ("Turret/SupplyCurrent_A",           turretMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber ("Turret/StatorCurrent_A",           turretMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber ("Turret/ClosedLoop_Error_MotorRot", turretMotor.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber ("Turret/CRT_Angle_Degrees",         getCRTAngleDegrees());
        SmartDashboard.putBoolean("Turret/At_Target",                 atTarget());
        SmartDashboard.putBoolean("Turret/CRT_Seeded",                crtSeeded);
        SmartDashboard.putBoolean("Turret/Chassis_Compensation",      chassisCompensationEnabled);
        SmartDashboard.putNumber ("Turret/Field_Target_Degrees",      fieldTargetDegrees);
        SmartDashboard.putNumber ("Turret/Chassis_Heading_Degrees",   chassisHeadingSupplier.get().getDegrees());
    }
}