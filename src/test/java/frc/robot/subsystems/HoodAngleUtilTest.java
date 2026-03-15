package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class HoodAngleUtilTest {

    @Test
    void computeOffsetMapsRawToDesiredAngle() {
        double rawAngleDeg = 36.298;
        double desiredAngleDeg = 18.2;

        double offsetDeg = HoodAngleUtil.computeSensorOffsetDeg(desiredAngleDeg, rawAngleDeg);
        double adjusted = HoodAngleUtil.applyOffset(rawAngleDeg, offsetDeg);

        assertEquals(desiredAngleDeg, adjusted, 1e-6);
    }

    @Test
    void rawRotationsRemoveOffset() {
        double desiredAngleDeg = 55.0;
        double offsetDeg = -18.098;

        double rawRotations = HoodAngleUtil.toRawRotations(desiredAngleDeg, offsetDeg);

        assertEquals((desiredAngleDeg - offsetDeg) / 360.0, rawRotations, 1e-9);
    }

    @Test
    void softLimitThresholdAppliesOffset() {
        double limitDeg = 56.2;
        double offsetDeg = -18.0;

        double threshold = HoodAngleUtil.toSoftLimitThreshold(limitDeg, offsetDeg);

        assertEquals((limitDeg - offsetDeg) / 360.0, threshold, 1e-9);
    }
}
