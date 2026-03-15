package frc.robot.subsystems;

final class HoodAngleUtil {
    private HoodAngleUtil() {
    }

    static double computeSensorOffsetDeg(double desiredAngleDeg, double rawAngleDeg) {
        return desiredAngleDeg - rawAngleDeg;
    }

    static double applyOffset(double rawAngleDeg, double offsetDeg) {
        return rawAngleDeg + offsetDeg;
    }

    static double toRawRotations(double desiredAngleDeg, double offsetDeg) {
        return (desiredAngleDeg - offsetDeg) / 360.0;
    }

    static double toSoftLimitThreshold(double limitDeg, double offsetDeg) {
        return (limitDeg - offsetDeg) / 360.0;
    }
}
