package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ShootingConstants;

public class ShootingCalculator {
    private static ShootingCalculator instance;

    // Filters to smooth the velocity of the mechanisms
    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));
    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));

    private Rotation2d lastTurretAngle;
    private double lastHoodAngle = Double.NaN;

    public static ShootingCalculator getInstance() {
        if (instance == null) instance = new ShootingCalculator();
        return instance;
    }

    /** * Container for the physical targets to feed to your subsystems.
     * All angles are already converted to degrees for easy implementation.
     */
    public record LaunchingParameters(
        boolean isValid,
        double turretAngleDegrees,
        double turretVelocityDegPerSec,
        double hoodAngleDegrees,
        double hoodVelocityDegPerSec,
        double flywheelRPM) {}

    private static final double minDistance = 1.34;
    private static final double maxDistance = 5.60;
    private static final double phaseDelay = 0.03;

    // Interpolation Maps
    private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    static {
        // Distance (meters) -> Hood Angle
        launchHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
        launchHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
        launchHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
        launchHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
        launchHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
        launchHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
        launchHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
        launchHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
        launchHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
        launchHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

        // Distance (meters) -> Flywheel RPM
        launchFlywheelSpeedMap.put(1.34, 210.0);
        launchFlywheelSpeedMap.put(1.78, 220.0);
        launchFlywheelSpeedMap.put(2.17, 220.0);
        launchFlywheelSpeedMap.put(2.81, 230.0);
        launchFlywheelSpeedMap.put(3.82, 250.0);
        launchFlywheelSpeedMap.put(4.09, 255.0);
        launchFlywheelSpeedMap.put(4.40, 260.0);
        launchFlywheelSpeedMap.put(4.77, 265.0);
        launchFlywheelSpeedMap.put(5.57, 275.0);
        launchFlywheelSpeedMap.put(5.60, 290.0);

        // Distance (meters) -> Time of flight (seconds)
        timeOfFlightMap.put(5.68, 1.16);
        timeOfFlightMap.put(4.55, 1.12);
        timeOfFlightMap.put(3.15, 1.11);
        timeOfFlightMap.put(1.88, 1.09);
        timeOfFlightMap.put(1.38, 0.90);
    }

    /**
     * Calculates the exact parameters needed to shoot on the move.
     *
     * @param estimatedPose The robot's current pose on the field.
     * @param robotVelocity The robot-relative speeds.
     * @param fieldVelocity The field-relative speeds.
     * @param target The X/Y coordinates of the Hub/Target.
     */
    public LaunchingParameters calculate(
            Pose2d estimatedPose, 
            ChassisSpeeds robotVelocity, 
            ChassisSpeeds fieldVelocity,
            Translation2d target) {

        // 1. Account for phase delay (latency compensation)
        estimatedPose = estimatedPose.exp(
            new Twist2d(
                robotVelocity.vxMetersPerSecond * phaseDelay,
                robotVelocity.vyMetersPerSecond * phaseDelay,
                robotVelocity.omegaRadiansPerSecond * phaseDelay));

        // 2. Find Turret Pose (offset from robot center)
        Transform2d robotToTurret2d = new Transform2d(
            ShootingConstants.ROBOT_TO_TURRET.getTranslation().toTranslation2d(),
            ShootingConstants.ROBOT_TO_TURRET.getRotation().toRotation2d()
        );
        Pose2d turretPosition = estimatedPose.transformBy(robotToTurret2d);
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        // 3. Imparted velocity calculations
        double robotAngle = estimatedPose.getRotation().getRadians();
        double turretVelocityX = fieldVelocity.vxMetersPerSecond
            + fieldVelocity.omegaRadiansPerSecond * (robotToTurret2d.getY() * Math.cos(robotAngle) - robotToTurret2d.getX() * Math.sin(robotAngle));
        double turretVelocityY = fieldVelocity.vyMetersPerSecond
            + fieldVelocity.omegaRadiansPerSecond * (robotToTurret2d.getX() * Math.cos(robotAngle) - robotToTurret2d.getY() * Math.sin(robotAngle));

        // 4. 20-iteration Lookahead Loop for Shooting on the Move
        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;
        
        for (int i = 0; i < 20; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            
            lookaheadPose = new Pose2d(
                turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                turretPosition.getRotation());
            
            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // 5. Final Hardware Target Conversions
        Rotation2d turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
        double hoodAngleRadians = launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
        
        if (lastTurretAngle == null) lastTurretAngle = turretAngle;
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngleRadians;

        double calculatedTurretVel = turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getDegrees() / 0.02);
        double calculatedHoodVel = hoodAngleFilter.calculate(
            Math.toDegrees(hoodAngleRadians - lastHoodAngle) / 0.02);

        lastTurretAngle = turretAngle;
        lastHoodAngle = hoodAngleRadians;

        boolean isValid = lookaheadTurretToTargetDistance >= minDistance 
                       && lookaheadTurretToTargetDistance <= maxDistance;

        // Telemetry
        SmartDashboard.putNumber("LaunchCalculator/LookaheadDist", lookaheadTurretToTargetDistance);
        SmartDashboard.putBoolean("LaunchCalculator/IsValidTarget", isValid);

        return new LaunchingParameters(
            isValid,
            turretAngle.getDegrees(),
            calculatedTurretVel,
            Math.toDegrees(hoodAngleRadians),
            calculatedHoodVel,
            launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance)
        );
    }
}