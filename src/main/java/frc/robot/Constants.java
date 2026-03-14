package frc.robot;

public final class Constants {

    public static final class IntakeConstants {

        // ── CAN ───────────────────────────────────────────────────────────────
        public static final int UPPER_INDEX_MOTOR_ID = 31;
        public static final int BOTTOM_ROLLER_MOTOR_ID = 32;
        public static final int PIVOT_MOTOR_ID = 33;
        public static final int OUTER_ROLLER_MOTOR_ID = 34;

        // ── Motor Inverts ─────────────────────────────────────────────────────
        public static final boolean UPPER_INDEX_INVERTED = false;
        public static final boolean BOTTOM_ROLLER_INVERTED = false;
        public static final boolean PIVOT_INVERTED = false;
        public static final boolean OUTER_ROLLER_INVERTED = false;

        // ── Current Limits (A) ────────────────────────────────────────────────
        public static final int ROLLER_SMART_CURRENT_LIMIT_A = 40;
        public static final int PIVOT_SMART_CURRENT_LIMIT_A = 30;

        // ── Roller Output Setpoints ───────────────────────────────────────────
        public static final double UPPER_INDEX_INTAKE_OUTPUT = 0.65;
        public static final double BOTTOM_ROLLER_INTAKE_OUTPUT = 0.70;
        public static final double OUTER_ROLLER_INTAKE_OUTPUT = 0.75;

        public static final double UPPER_INDEX_OUTTAKE_OUTPUT = -0.65;
        public static final double BOTTOM_ROLLER_OUTTAKE_OUTPUT = -0.70;
        public static final double OUTER_ROLLER_OUTTAKE_OUTPUT = -0.75;

        // ── Pivot Gearing ─────────────────────────────────────────────────────
        // 15:1 AM Sport planetary + 24:15 timing belt = 24:1 overall
        public static final double PIVOT_GEAR_RATIO = 15.0 * (24.0 / 15.0); // = 24.0

        // ── Pivot Angle Setpoints (mechanism degrees) ─────────────────────────
        // Boot-zero convention: robot starts physically in Closed position.
        public static final double PIVOT_CLOSED_ANGLE_DEG = 0.0;
        public static final double PIVOT_OPEN_ANGLE_DEG   = (25.0 / PIVOT_GEAR_RATIO) * 360.0; // ≈ 375° ⚠️ verify on hardware

        // ── Pivot Soft/Hard Limits (mechanism degrees) ────────────────────────
        public static final double PIVOT_SOFT_MIN_ANGLE_DEG = -5.0;
        public static final double PIVOT_SOFT_MAX_ANGLE_DEG = 380.0;
        public static final double PIVOT_HARD_MIN_ANGLE_DEG = -10.0;
        public static final double PIVOT_HARD_MAX_ANGLE_DEG = 390.0;
        public static final double PIVOT_AT_TARGET_TOLERANCE_DEG = 2.0;

        // ── YAMS Closed-Loop Gains (WPILib ProfiledPIDController in degrees) ──
        public static final double PIVOT_kP = 0.03; // ⚠️ tune on hardware/sim
        public static final double PIVOT_kI = 0.0;
        public static final double PIVOT_kD = 0.0;

        // ── ArmFeedforward ────────────────────────────────────────────────────
        public static final double PIVOT_kS = 0.0; // ⚠️ static friction V — tune on hardware
        public static final double PIVOT_kG = 0.0; // ⚠️ gravity V — tune on hardware
        public static final double PIVOT_kV = 0.0; // ⚠️ velocity V/(rad/s) — tune on hardware

        // ── Pivot Motion Constraints ──────────────────────────────────────────
        // Derived from old Smart Motion limits converted through 24:1 ratio:
        // 1500 motor RPM / 24 * 6 = 375 deg/s; 2000 RPM/s / 24 * 6 = 500 deg/s²
        public static final double PIVOT_MAX_VEL_DEG_PER_SEC    = 375.0;
        public static final double PIVOT_MAX_ACCEL_DEG_PER_SEC2 = 500.0;

        // ── Arm Physical Parameters (simulation accuracy) ─────────────────────
        public static final double PIVOT_ARM_LENGTH_METERS = 0.35; // ⚠️ estimate — measure actual
        public static final double PIVOT_ARM_MASS_KG       = 1.2;  // ⚠️ estimate — weigh actual
    }

    public static final class TurretConstants {

        // ── CAN ───────────────────────────────────────────────────────────────
        public static final int    TURN_MOTOR_ID  = 21;
        public static final int    ENCODER_10T_ID = 22;
        public static final int    ENCODER_17T_ID = 23;
        public static final String CANBUS         = "rio";

        // ── Gearing ───────────────────────────────────────────────────────────
        public static final int    TURRET_RING_GEAR_TEETH   = 85;
        public static final int    ENCODER_10T_PINION_TEETH = 10;
        public static final int    ENCODER_17T_PINION_TEETH = 17;

        public static final double MOTOR_TO_MECHANISM_RATIO = (48.0 / 12.0) * (85.0 / 10.0); // = 34.0

        // ── Motor ─────────────────────────────────────────────────────────────
        public static final boolean MOTOR_INVERTED       = false;
        public static final double  SUPPLY_CURRENT_LIMIT = 40.0;
        public static final double  STATOR_CURRENT_LIMIT = 80.0;
        public static final double  MAX_OUTPUT           = 0.05; // ⚠️ 5% — increase per speed guide

        // ── PID / Feedforward (Slot 0) ────────────────────────────────────────
        public static final double kP = 5.0;  // ⚠️ start low — tune up per speed guide
        public static final double kI = 0.0;
        public static final double kD = 0.0;  // ⚠️ add only if oscillation occurs
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.0;  // ⚠️ leave at 0 until speed stage 3+

        // ── MotionMagic ───────────────────────────────────────────────────────
        // Edit TURRET_CRUISE_DEG_PER_SEC only — motor RPS derived automatically
        public static final double TURRET_CRUISE_DEG_PER_SEC = 180.0;  // ⚠️ increase per speed guide
        public static final double TURRET_ACCEL_DEG_PER_SEC2 = 360.0;  // always 2× cruise
        public static final double TURRET_JERK_DEG_PER_SEC3  = 1800.0; // always 10× cruise

        public static final double MM_CRUISE_VELOCITY =
            (TURRET_CRUISE_DEG_PER_SEC / 360.0) * MOTOR_TO_MECHANISM_RATIO;
        public static final double MM_ACCELERATION =
            (TURRET_ACCEL_DEG_PER_SEC2 / 360.0) * MOTOR_TO_MECHANISM_RATIO;
        public static final double MM_JERK =
            (TURRET_JERK_DEG_PER_SEC3  / 360.0) * MOTOR_TO_MECHANISM_RATIO;

        // ── Mechanism Range ───────────────────────────────────────────────────
        public static final double MIN_ANGLE_ROT = -0.5;
        public static final double MAX_ANGLE_ROT =  0.5;

        // ── CRT ───────────────────────────────────────────────────────────────
        public static final double ENCODER_10T_OFFSET_ROT  = 0.0;
        public static final double ENCODER_17T_OFFSET_ROT  = 0.0;
        public static final double CRT_MATCH_TOLERANCE_ROT = 0.06;

        // ── Control ───────────────────────────────────────────────────────────
        public static final double AT_TARGET_TOLERANCE_DEG = 1.0;
    }
}