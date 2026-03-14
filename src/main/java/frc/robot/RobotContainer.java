// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final TurretSubsystem turret = new TurretSubsystem(() -> drivetrain.getState().Pose.getRotation());
    private final IntakeSubsystem intake = new IntakeSubsystem();


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        configureTurretBindings();
        configureIntakeBindings();
    }


private void configureTurretBindings() {

    // ── Open Loop Direction Test (Step 5 — remove after testing) ─────────
    joystick.start().and(joystick.povUp())
        .whileTrue(Commands.run(() -> turret.testOpenLoop(0.05), turret))
        .onFalse(Commands.runOnce(() -> turret.stop(), turret));

    joystick.start().and(joystick.povDown())
        .whileTrue(Commands.run(() -> turret.testOpenLoop(-0.05), turret))
        .onFalse(Commands.runOnce(() -> turret.stop(), turret));

    // ── Heading Presets ───────────────────────────────────────────────────
    joystick.x()
        .onTrue(Commands.runOnce(() -> turret.setHeading(0), turret));    // forward

    joystick.y()
        .onTrue(Commands.runOnce(() -> turret.setHeading(90), turret));   // right

    joystick.leftBumper()
        .onTrue(Commands.runOnce(() -> turret.setHeading(-90), turret));  // left

    joystick.rightBumper()
        .onTrue(Commands.runOnce(() -> turret.setHeading(180), turret));  // rear

    // ── Relative Rotation ─────────────────────────────────────────────────
    joystick.povRight()
        .onTrue(Commands.runOnce(() -> turret.rotateBy(5), turret));      // +5°

    joystick.povLeft()
        .onTrue(Commands.runOnce(() -> turret.rotateBy(-5), turret));     // -5°

    // ── Chassis Compensation ──────────────────────────────────────────────
    joystick.rightStick()
        .whileTrue(Commands.run(() -> turret.setFieldHeading(
            turret.getMechanismDegrees()
        ), turret));

    joystick.leftStick()
                .onTrue(Commands.runOnce(() -> turret.disableChassisCompensation(), turret));


    // ── Stop ──────────────────────────────────────────────────────────────
    joystick.back()
        .onTrue(Commands.runOnce(() -> turret.stop(), turret));
}

private void configureIntakeBindings() {
    // ── Roller Control ──────────────────────────────────────────────────────
    joystick.leftTrigger()
        .whileTrue(Commands.run(intake::intake, intake))
        .onFalse(Commands.runOnce(intake::stopRollers, intake));

    joystick.rightTrigger()
        .whileTrue(Commands.run(intake::outtake, intake))
        .onFalse(Commands.runOnce(intake::stopRollers, intake));

    // ── Pivot State Control (gated with BACK to avoid accidental conflict) ─
    joystick.back().and(joystick.povUp())
        .onTrue(Commands.runOnce(intake::open, intake));

    joystick.back().and(joystick.povDown())
        .onTrue(Commands.runOnce(intake::close, intake));

    joystick.back().and(joystick.povLeft())
        .onTrue(Commands.runOnce(intake::stopPivot, intake));

    // Emergency stop both intake and pivot.
    joystick.back().and(joystick.rightTrigger())
        .onTrue(Commands.runOnce(() -> {
            intake.stopRollers();
            intake.stopPivot();
        }, intake));
}

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
