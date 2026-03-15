package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.generated.TunerConstants;

public class RobotContainer {

    // --- Speeds ---
    private final double MaxSpeed       = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // --- Swerve ---
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // --- Controller ---
    private final CommandXboxController joystick = new CommandXboxController(0);

    // --- Subsystems ---
    public  final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final TurretSubsystem         turret     = new TurretSubsystem(() -> drivetrain.getState().Pose.getRotation());
    public final IntakeSubsystem         intake     = new IntakeSubsystem();
    public final IntakeArmSubsystem      intakeArm  = new IntakeArmSubsystem();
    public final FlywheelSubsystem       flywheel   = new FlywheelSubsystem();
    public final HoodSubsystem           hood       = new HoodSubsystem();

    // --- Auto ---
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        SmartDashboard.putNumber("Turret Target Angle (deg)", 0.0);
        SmartDashboard.putNumber("Hood Target Angle (deg)", Constants.HoodConstants.HOOD_MIN_ANGLE_DEG);

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {

        // --- Drivetrain Default ---
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                     .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                     .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        configureDriveBindings();
        configureTurretBindings();
        configureIntakeBindings();
        configureShooterBindings();
    }

    private void configureDriveBindings() {
        // Reset field-centric heading
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    private void configureTurretBindings() {
        // Go to SmartDashboard angle
        joystick.x().onTrue(Commands.runOnce(() -> {
            double angle = SmartDashboard.getNumber("Turret Target Angle (deg)", 0.0);
            turret.setHeading(angle);
        }, turret));

        // Stop turret
        joystick.back().onTrue(Commands.runOnce(turret::stop, turret));
    }

    private void configureIntakeBindings() {
        // Toggle intake arm open/close
        joystick.y().onTrue(Commands.runOnce(intakeArm::toggleArm, intakeArm));

        // Run rollers in (hold)
        joystick.leftTrigger()
            .whileTrue(Commands.run(intake::intake, intake))
            .onFalse(Commands.runOnce(intake::stopRollers, intake));

        // Run rollers out (hold)
        joystick.rightTrigger()
            .whileTrue(Commands.run(intake::outtake, intake))
            .onFalse(Commands.runOnce(intake::stopRollers, intake));
    }

    private void configureShooterBindings() {
    // ── Flywheel ──────────────────────────────────────────────────────────
    joystick.a()
        .whileTrue(Commands.run(flywheel::shoot, flywheel))
        .onFalse(Commands.runOnce(flywheel::stop, flywheel));

    joystick.b()
        .whileTrue(Commands.run(flywheel::warmUp, flywheel))
        .onFalse(Commands.runOnce(flywheel::stop, flywheel));

    // ── Hood ──────────────────────────────────────────────────────────────
    // Toggle PID — no subsystem requirement so POV bindings never interrupt it
    joystick.rightBumper().onTrue(Commands.runOnce(() -> {
        if (hood.isPIDEnabled()) {
            hood.disablePID();
        } else {
            hood.enablePID();
        }
    }));

    // Set hood to SmartDashboard angle
    joystick.povUp().onTrue(Commands.runOnce(() -> {
        double angle = SmartDashboard.getNumber("Hood Target Angle (deg)",
            Constants.HoodConstants.HOOD_MIN_ANGLE_DEG);
        hood.setAngle(angle);
    }, hood));

    // Re-seed hood position at minimum angle (move to 18.2° before pressing)
    joystick.povDown().onTrue(Commands.runOnce(() ->
        hood.seedPositionDeg(Constants.HoodConstants.HOOD_MIN_ANGLE_DEG), hood));

}

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}