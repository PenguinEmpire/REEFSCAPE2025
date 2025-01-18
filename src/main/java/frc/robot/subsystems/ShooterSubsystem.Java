package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // update with actual ids
    private static final int MOTOR_ID = 11;

    //update with actual channel
    private static final int INFRARED_SENSOR_CHANNEL = 0;

    // Motor power levels
    private static final double SHOOT_POWER = 1.0; // tune and update
    private static final double SUCK_POWER = -0.5; 

    // Motor controller
    private final CANSparkMax shooterMotor;

    // Infrared sensor
    private final DigitalInput infraredSensor;

    public ShooterSubsystem() {
        // Initialize the motor controller
        shooterMotor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);

        // Configure motor settings
        shooterMotor.setSmartCurrentLimit(40); // test for optimal amount

        // Initialize the infrared sensor
        infraredSensor = new DigitalInput(INFRARED_SENSOR_CHANNEL);

        
        SmartDashboard.putNumber("Shooter Shoot Power", SHOOT_POWER);
        SmartDashboard.putNumber("Shooter Suck Power", SUCK_POWER);
    }

    /**
     * Shoots the coral out by running the motor forward at full power.
     */
    public void shoot() {
        double power = SmartDashboard.getNumber("Shooter Shoot Power", SHOOT_POWER);
        shooterMotor.set(power);
    }

    /**
     * Sucks the coral in by running the motor backward at half power,
     * but stops if the infrared sensor detects the coral.
     */
    public void suck() {
        if (!isCoralDetected()) {
            double power = SmartDashboard.getNumber("Shooter Suck Power", SUCK_POWER);
            shooterMotor.set(power);
        } else {
            stop(); // Stop the motor if the coral is detected
        }
    }

    /**
     * Stops the motor.
     */
    public void stop() {
        shooterMotor.set(0.0);
    }

    /**
     * Checks if the infrared sensor detects the coral.
     *
     * @return True if the coral is detected, false otherwise.
     */
    public boolean isCoralDetected() {
        return !infraredSensor.get(); // Assuming the sensor is normally open
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral Detected", isCoralDetected());
        SmartDashboard.putNumber("Shooter Motor Power", shooterMotor.getAppliedOutput());
    }
}
