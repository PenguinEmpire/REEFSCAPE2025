package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
    // Shooter motor
    private final CANSparkMax shooterMotor;

    // Limit switch
    private final DigitalInput limitSwitch;

    public ShooterSubsystem() {
        // Initialize the shooter motor
        shooterMotor = new CANSparkMax(Shooter.MOTOR_ID, MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();

        // Initialize the limit switch
        limitSwitch = new DigitalInput(Shooter.LIMIT_SWITCH_CHANNEL);
        // Add shooter motor power to SmartDashboard for testing
        SmartDashboard.putNumber("Shooter Power", Shooter.FORWARD_POWER);
    }

    /**
     * Spins the shooter motor in the forward direction.
     */
    public void shootForward() {
        double power = SmartDashboard.getNumber("Shooter Power", Shooter.FORWARD_POWER);
        shooterMotor.set(power);
    }

    /**
     * Spins the shooter motor in the reverse direction.
     */
    public void shootReverse() {
        shooterMotor.set(Shooter.REVERSE_POWER);
    }

    /**
     * Stops the shooter motor.
     */
    public void stopShooter() {
        shooterMotor.set(Shooter.STOP_POWER);
    }

    /**
     * Checks if the limit switch is triggered.
     *
     * @return True if the limit switch is pressed, false otherwise.
     */
    public boolean isLimitSwitchTriggered() {
        return !limitSwitch.get(); // Assuming the limit switch is normally open
    }

    @Override
    public void periodic() {
        // Stop the shooter if the limit switch is triggered
        if (isLimitSwitchTriggered()) {
            stopShooter();

            // Log to SmartDashboard
            SmartDashboard.putBoolean("Shooter Limit Switch Triggered", true);
        } else {
            SmartDashboard.putBoolean("Shooter Limit Switch Triggered", false);
        }

        // Update SmartDashboard with shooter status
        SmartDashboard.putNumber("Shooter Current Power", shooterMotor.get());
    }
}
