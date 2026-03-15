package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
    private final SparkMax feederMotor;

    public FeederSubsystem() {
        feederMotor = new SparkMax(FeederConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
        
        // Configure the SparkMax
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(FeederConstants.MOTOR_INVERTED);
        config.smartCurrentLimit(FeederConstants.CURRENT_LIMIT);
        
        // Apply configuration
        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Runs the funnel to carry the ball up to the shooter. */
    public void runFeeder() {
        feederMotor.set(FeederConstants.FEEDER_SPEED);
    }

    /** Runs the funnel in reverse (to clear a jam). */
    public void reverseFeeder() {
        feederMotor.set(-FeederConstants.FEEDER_SPEED);
    }

    /** Stops the funnel. */
    public void stop() {
        feederMotor.set(0);
    }

    @Override
    public void periodic() {
        // You can add SmartDashboard telemetry here if needed
    }
}