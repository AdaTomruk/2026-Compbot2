package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShootingConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShootingCalculator;
import frc.robot.subsystems.ShootingCalculator.LaunchingParameters;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Simplified auto-aim that only points the turret at the target.
 */
public class AutoAimTurretCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final TurretSubsystem turret;
    private final ShootingCalculator calculator;

    public AutoAimTurretCommand(CommandSwerveDrivetrain drivetrain, TurretSubsystem turret) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.calculator = ShootingCalculator.getInstance();

        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose2d pose = drivetrain.getState().Pose;
        ChassisSpeeds robotSpeeds = drivetrain.getState().Speeds;
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, pose.getRotation());

        Translation2d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? ShootingConstants.RED_TARGET
            : ShootingConstants.BLUE_TARGET;

        LaunchingParameters params = calculator.calculate(pose, robotSpeeds, fieldSpeeds, target);

        if (params.isValid()) {
            turret.setFieldHeading(params.turretAngleDegrees());
        } else {
            turret.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
